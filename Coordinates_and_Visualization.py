import os
import sys
import time
import sumolib
import traci
import airsim
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class CoordinateTransformer:
    """SUMO到AirSim的坐标转换器"""

    def __init__(self, origin_offset=(0, 0, 0), rotation_angle=0):
        """
        :param origin_offset: 原点偏移量 [x, y, z]
        :param rotation_angle: 平面旋转角度（度）
        """
        self.origin_offset = np.array(origin_offset)
        self.rotation_angle = np.radians(rotation_angle)

    def sumo_to_airsim(self, sumo_pos):
        """
        将SUMO坐标(x,y,z)转换为AirSim坐标(NED坐标系)
        :param sumo_pos: [x, y, z] SUMO坐标系(ENU)
        :return: [x, y, z] AirSim坐标系(NED)
        """
        # 1. 处理高度（SUMO的z通常为0，AirSim的z向下为负）
        z = -sumo_pos[2] if len(sumo_pos) > 2 else 0

        # 2. 平面旋转（可选）
        rot_matrix = np.array([
            [np.cos(self.rotation_angle), -np.sin(self.rotation_angle)],
            [np.sin(self.rotation_angle), np.cos(self.rotation_angle)]
        ])
        x, y = np.dot(rot_matrix, [sumo_pos[0], sumo_pos[1]])

        # 3. 应用原点偏移和坐标系转换
        ned_x = y + self.origin_offset[0]  # SUMO的y对应AirSim的x（北）
        ned_y = x + self.origin_offset[1]  # SUMO的x对应AirSim的y（东）
        ned_z = z + self.origin_offset[2]

        return [ned_x, ned_y, ned_z]


class SUMOAirSimVisualizer:
    def __init__(self, sumo_config_path):
        """
        初始化SUMO-AirSim可视化工具
        :param sumo_config_path: SUMO配置文件路径
        """
        self.sumo_config_path = sumo_config_path
        self.transformer = CoordinateTransformer(origin_offset=[0, 0, 0], rotation_angle=0)
        self.vehicle_data = {}

        # 初始化AirSim客户端
        self.airsim_client = airsim.CarClient()
        self.airsim_client.confirmConnection()
        print("AirSim中的车辆:", self.airsim_client.listVehicles())

    def start_simulation(self):
        """启动SUMO模拟并同步到AirSim"""
        if 'SUMO_HOME' not in os.environ:
            sys.exit("请设置SUMO_HOME环境变量")

        sumo_binary = sumolib.checkBinary('sumo')
        traci.start([sumo_binary, "-c", self.sumo_config_path])

        print("SUMO-AirSim同步开始...")
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()
            vehicles = traci.vehicle.getIDList()

            for veh_id in vehicles:
                if veh_id not in self.vehicle_data:
                    self.vehicle_data[veh_id] = {
                        'sumo_pos': [],
                        'airsim_pos': [],
                        'time': []
                    }

                # 获取SUMO坐标
                sumo_pos = list(traci.vehicle.getPosition(veh_id)) + [0]  # [x, y, z=0]

                # 转换到AirSim坐标
                airsim_pos = self.transformer.sumo_to_airsim(sumo_pos)

                # 存储数据
                self.vehicle_data[veh_id]['sumo_pos'].append(sumo_pos)
                self.vehicle_data[veh_id]['airsim_pos'].append(airsim_pos)
                self.vehicle_data[veh_id]['time'].append(current_time)
                self._sync_to_airsim(veh_id, airsim_pos)

        traci.close()
        print(f"模拟完成，共收集{len(self.vehicle_data)}辆车的轨迹数据")

    def _sync_to_airsim(self, veh_id, pos):
        try:
            pose = airsim.Pose(
                position_val=airsim.Vector3r(float(pos[0]), float(pos[1]), float(pos[2])),
                orientation_val=airsim.to_quaternion(0, 0, 0)
            )
            return self.airsim_client.simSetVehiclePose(
                pose,
                ignore_collision=True,
                vehicle_name=veh_id
            )
        except Exception as e:
            print(f"同步错误 {veh_id}: {type(e).__name__}: {str(e)}")
            return False

    def plot_coordinate_comparison(self):
        """绘制SUMO和AirSim坐标对比图"""
        plt.figure(figsize=(14, 6))

        # SUMO原始坐标
        plt.subplot(1, 2, 1)
        for veh_id, data in self.vehicle_data.items():
            sumo_arr = np.array(data['sumo_pos'])
            plt.plot(sumo_arr[:, 0], sumo_arr[:, 1], label=f'Vehicle {veh_id}')
        plt.title("SUMO Coordinates (ENU)")
        plt.xlabel("East (X)")
        plt.ylabel("North (Y)")
        plt.grid(True)
        plt.legend()

        # AirSim转换后坐标
        plt.subplot(1, 2, 2)
        for veh_id, data in self.vehicle_data.items():
            airsim_arr = np.array(data['airsim_pos'])
            plt.plot(airsim_arr[:, 0], airsim_arr[:, 1], '--', label=f'Vehicle {veh_id}')
        plt.title("AirSim Coordinates (NED)")
        plt.xlabel("North (X)")
        plt.ylabel("East (Y)")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

    def animate_movement(self):
        """动态展示坐标转换过程"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        lines = {}

        # 初始化绘图元素
        for veh_id in self.vehicle_data.keys():
            # SUMO坐标图
            line_sumo, = ax1.plot([], [], 'o-', label=f'Vehicle {veh_id}')
            # AirSim坐标图
            line_airsim, = ax2.plot([], [], 'o--', label=f'Vehicle {veh_id}')
            lines[veh_id] = (line_sumo, line_airsim)

        def update(frame):
            for veh_id, (line_sumo, line_airsim) in lines.items():
                data = self.vehicle_data[veh_id]
                # 更新SUMO坐标轨迹
                sumo_arr = np.array(data['sumo_pos'][:frame + 1])
                line_sumo.set_data(sumo_arr[:, 0], sumo_arr[:, 1])
                # 更新AirSim坐标轨迹
                airsim_arr = np.array(data['airsim_pos'][:frame + 1])
                line_airsim.set_data(airsim_arr[:, 0], airsim_arr[:, 1])

            ax1.set_title(f"SUMO Coordinates (Time: {data['time'][frame]:.1f}s)")
            ax2.set_title(f"AirSim Coordinates (Time: {data['time'][frame]:.1f}s)")
            return [line for pair in lines.values() for line in pair]

        # 设置坐标轴范围
        all_sumo = np.concatenate([d['sumo_pos'] for d in self.vehicle_data.values()])
        all_airsim = np.concatenate([d['airsim_pos'] for d in self.vehicle_data.values()])
        ax1.set_xlim(np.min(all_sumo[:, 0]) - 10, np.max(all_sumo[:, 0]) + 10)
        ax1.set_ylim(np.min(all_sumo[:, 1]) - 10, np.max(all_sumo[:, 1]) + 10)
        ax2.set_xlim(np.min(all_airsim[:, 0]) - 10, np.max(all_airsim[:, 0]) + 10)
        ax2.set_ylim(np.min(all_airsim[:, 1]) - 10, np.max(all_airsim[:, 1]) + 10)

        ax1.set_xlabel("East (X)");
        ax1.set_ylabel("North (Y)")
        ax2.set_xlabel("North (X)");
        ax2.set_ylabel("East (Y)")
        ax1.grid(True);
        ax2.grid(True)
        ax1.legend();
        ax2.legend()

        ani = FuncAnimation(fig, update, frames=len(self.vehicle_data[list(self.vehicle_data.keys())[0]]['time']),
                            interval=100, blit=True)
        plt.tight_layout()
        plt.show()
        return ani


def main():
    sumo_config_path = "D:\\SUMO\\crossing.sumocfg"

    visualizer = SUMOAirSimVisualizer(sumo_config_path)
    visualizer.start_simulation()

    while not visualizer.vehicle_data:
        print("等待模拟数据...")
        time.sleep(1)
    while True:
        print("\n请选择可视化方式:")
        print("1. 静态坐标对比图")
        print("2. 动态轨迹动画")
        print("3. 退出")

        choice = input("请输入选项(1/2/3): ").strip()

        if choice == '1':
            visualizer.plot_coordinate_comparison()
        elif choice == '2':
            anim = visualizer.animate_movement()
            plt.show()
        elif choice == '3':
            break
        else:
            print("无效输入，请重新选择")


if __name__ == "__main__":
    main()