import airsim
import numpy as np
import time
import threading
from math import sqrt, atan2, degrees


class ConflictAwareDrone:
    def __init__(self, drone_name="Drone1", start_pos=[0, 0, -5], color=[255, 0, 0]):
        """初始化单个无人机"""
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.name = drone_name
        self.client.enableApiControl(True, vehicle_name=self.name)
        self.client.armDisarm(True, vehicle_name=self.name)

        # 设置初始位置和颜色（可视化区分）
        pose = airsim.Pose(airsim.Vector3r(*start_pos), airsim.Quaternionr())
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.name)

        self.client.simSetSegmentationObjectID(self.name, color[0] * 65536 + color[1] * 256 + color[2])

        # 航线参数
        self.path = []
        self.current_waypoint = 0
        self.speed = 3  # m/s
        self.safety_radius = 2  # 防撞安全半径
        self.replan_lock = threading.Lock()

    def takeoff(self, altitude=5):
        """起飞到指定高度"""
        self.client.takeoffAsync(vehicle_name=self.name).join()
        self.client.moveToZAsync(-altitude, self.speed, vehicle_name=self.name).join()

    def land(self):
        """降落"""
        self.client.landAsync(vehicle_name=self.name).join()
        self.client.armDisarm(False, vehicle_name=self.name)

    def set_mission_path(self, path):
        """设置目标航点路径（列表[[x,y,z]]）"""
        self.path = path.copy()
        self.current_waypoint = 0

    def get_position(self):
        """获取当前NED坐标"""
        pos = self.client.getMultirotorState(vehicle_name=self.name).kinematics_estimated.position
        return [pos.x_val, pos.y_val, pos.z_val]

    def get_neighbor_drones(self, all_drones):
        """获取周围其他无人机状态（位置+速度）"""
        neighbors = []
        for drone in all_drones:
            if drone.name != self.name:
                state = self.client.getMultirotorState(vehicle_name=drone.name)
                pos = state.kinematics_estimated.position
                vel = state.kinematics_estimated.linear_velocity
                neighbors.append({
                    "position": [pos.x_val, pos.y_val, pos.z_val],
                    "velocity": [vel.x_val, vel.y_val, vel.z_val],
                    "name": drone.name
                })
        return neighbors

    def detect_conflict(self, neighbors, lookahead_time=3):
        """冲突检测"""
        my_pos = np.array(self.get_position())
        my_vel = np.array(self.calculate_desired_velocity())

        for neighbor in neighbors:
            other_pos = np.array(neighbor["position"])
            other_vel = np.array(neighbor["velocity"])

            rel_pos = other_pos - my_pos
            rel_vel = other_vel - my_vel

            t_min = -np.dot(rel_pos, rel_vel) / (np.linalg.norm(rel_vel) ** 2 + 1e-6)
            t_min = max(0, min(lookahead_time, t_min))
            closest_dist = np.linalg.norm(rel_pos + rel_vel * t_min)

            if closest_dist < self.safety_radius:
                print(f"[{self.name}] Conflict detected with {neighbor['name']} (D={closest_dist:.2f}m)")
                return True
        return False

    def calculate_desired_velocity(self):
        """计算当前航点方向的目标速度矢量"""
        if self.current_waypoint >= len(self.path):
            return [0, 0, 0]

        target = np.array(self.path[self.current_waypoint])
        current = np.array(self.get_position())
        direction = target - current
        dist = np.linalg.norm(direction)

        if dist < 1.0:  # 到达航点
            self.current_waypoint += 1
            return self.calculate_desired_velocity()

        return (direction / dist) * self.speed

    def avoid_conflict(self, neighbors):
        """冲突避免策略（优先级规则+垂直避让）"""
        my_priority = int(self.name.replace("Drone", ""))

        for neighbor in neighbors:
            other_priority = int(neighbor["name"].replace("Drone", ""))
            if other_priority < my_priority:  # 低优先级无人机避让
                print(f"[{self.name}] Yielding to higher-priority {neighbor['name']}")
                return [0, 0, -1]  # 垂直下降避让

        # 默认水平绕行（向右）
        return [0, 1, 0]

    def execute_mission(self, all_drones):
        """执行航线任务（含动态冲突处理）"""
        while self.current_waypoint < len(self.path):
            # 计算理想速度
            desired_vel = self.calculate_desired_velocity()

            # 检测冲突
            neighbors = self.get_neighbor_drones(all_drones)
            if self.detect_conflict(neighbors):
                avoidance_vel = self.avoid_conflict(neighbors)
                actual_vel = np.array(desired_vel) + np.array(avoidance_vel)
            else:
                actual_vel = desired_vel

            # 执行运动
            self.client.moveByVelocityZAsync(
                actual_vel[0],
                actual_vel[1],
                self.get_position()[2],
                0.5,
                vehicle_name=self.name
            )
            time.sleep(0.1)

        print(f"[{self.name}] Mission complete!")
        self.land()


class MultiDroneCoordinator:
    def __init__(self, drone_count=3):
        """初始化多无人机系统"""
        self.drones = []
        self.setup_drones(drone_count)

    def setup_drones(self, count):
        """创建无人机并设置不同颜色和初始位置"""
        colors = [
            [255, 0, 0],  # 红
            [0, 255, 0],  # 绿
            [0, 0, 255],  # 蓝
            [255, 255, 0],  # 黄
        ]

        start_positions = [
            [0, 0, -5],
            [5, 0, -5],
            [0, 5, -5],
        ]

        for i in range(count):
            drone = ConflictAwareDrone(
                drone_name=f"Drone{i + 1}",
                start_pos=start_positions[i % len(start_positions)],
                color=colors[i % len(colors)]
            )
            self.drones.append(drone)

    def assign_missions(self):
        """设置交叉测试航线"""
        # 航线设计：两两交叉
        paths = [
            [[20, 0, -5], [20, 20, -5]],  # Drone1: 右→右上
            [[0, 20, -5], [20, 20, -5]],  # Drone2: 上→右上
            [[10, -10, -5], [10, 30, -5]],  # Drone3: 垂直穿越
        ]

        for i, drone in enumerate(self.drones):
            if i < len(paths):
                drone.set_mission_path(paths[i])

    def start_missions(self):
        """多线程启动所有无人机任务"""
        threads = []
        for drone in self.drones:
            drone.takeoff()
            time.sleep(1)

            t = threading.Thread(
                target=drone.execute_mission,
                args=([d for d in self.drones if d != drone],))
            t.start()
            threads.append(t)

            for t in threads:
                t.join()

if __name__ == "__main__":
    coordinator = MultiDroneCoordinator(drone_count=3)
    coordinator.assign_missions()

    print("Starting multi-drone mission with conflict avoidance...")
    coordinator.start_missions()

    print("All missions completed!")