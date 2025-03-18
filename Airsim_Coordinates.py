import airsim
import keyboard
import time
import threading


class AirSimDroneController:
    def __init__(self):
        """初始化 AirSim 客户端并连接到服务器"""
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)  # 启用 API 控制
        self.client.armDisarm(True)  # 解锁无人机
        print("Drone initialized and ready to fly.")

    def takeoff(self):
        """控制无人机起飞"""
        print("Taking off...")
        self.client.takeoffAsync().join()
        print("Drone is airborne.")

    def move_drone(self, key, duration=0.5, speed=1.0):
        """根据键盘输入控制无人机运动"""
        if key == 'w':  # 向前
            self.client.moveByVelocityZAsync(0, speed,
                                             self.client.getMultirotorState().kinematics_estimated.position.z_val,
                                             duration)
        elif key == 's':  # 向后
            self.client.moveByVelocityZAsync(0, -speed,
                                             self.client.getMultirotorState().kinematics_estimated.position.z_val,
                                             duration)
        elif key == 'a':  # 向左
            self.client.moveByVelocityZAsync(-speed, 0,
                                             self.client.getMultirotorState().kinematics_estimated.position.z_val,
                                             duration)
        elif key == 'd':  # 向右
            self.client.moveByVelocityZAsync(speed, 0,
                                             self.client.getMultirotorState().kinematics_estimated.position.z_val,
                                             duration)
        elif key == 'q':  # 上升
            self.client.moveByVelocityZAsync(0, 0, -speed, duration)
        elif key == 'e':  # 下降
            self.client.moveByVelocityZAsync(0, 0, speed, duration)
        else:
            print("Invalid key. Use W, A, S, D, Q, E to control the drone.")

    def land(self):
        """控制无人机降落"""
        print("Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)  # 锁定无人机
        self.client.enableApiControl(False)  # 关闭 API 控制
        print("Drone landed and control disabled.")

    def get_drone_state(self):
        """获取并打印无人机的当前坐标和速度矢量"""
        state = self.client.getMultirotorState()
        position = state.kinematics_estimated.position
        velocity = state.kinematics_estimated.linear_velocity
        print(f"NED Position: X={position.x_val:.2f}, Y={position.y_val:.2f}, Z={position.z_val:.2f}")
        print(f"Velocity Vector: X={velocity.x_val:.2f}, Y={velocity.y_val:.2f}, Z={velocity.z_val:.2f}")

    def monitor_drone_state(self, interval=0.5):
        """定时任务：每隔指定时间间隔获取并打印无人机状态"""
        while not self.stop_monitoring:
            self.get_drone_state()
            time.sleep(interval)

    def start_monitoring(self):
        self.stop_monitoring = False
        threading.Thread(target=self.monitor_drone_state).start()

    def stop_monitoring(self):
        self.stop_monitoring = True

    def __del__(self):
        self.land()


# 主程序
if __name__ == "__main__":
    drone = AirSimDroneController()
    drone.takeoff()

    # 启动定时任务，每隔 0.5 秒输出无人机状态
    drone.start_monitoring()

    print("Use W, A, S, D to control the drone. Press ESC to exit.")
    while True:
        if keyboard.is_pressed('esc'):
            break
        elif keyboard.is_pressed('w'):
            drone.move_drone('w')
        elif keyboard.is_pressed('s'):
            drone.move_drone('s')
        elif keyboard.is_pressed('a'):
            drone.move_drone('a')
        elif keyboard.is_pressed('d'):
            drone.move_drone('d')
        elif keyboard.is_pressed('q'):
            drone.move_drone('q')
        elif keyboard.is_pressed('e'):
            drone.move_drone('e')

        time.sleep(0.1)

    drone.stop_monitoring()
    drone.land()