import airsim
import keyboard
import time
from math import sqrt


class AdvancedAirSimDroneController:
    def __init__(self):
        """初始化连接并配置无人机"""
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.stop_mission = False
        print("Drone connected and ready.")

    def takeoff(self, altitude=5):
        """起飞到指定高度（米）"""
        print(f"Taking off to {altitude}m...")
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-altitude, 3).join()
        print("Takeoff complete.")

    def land(self):
        """安全降落"""
        print("Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        print("Drone landed.")

    def move_by_velocity(self, vx, vy, vz, duration):
        """速度控制（保持高度10m）"""
        self.client.moveByVelocityZAsync(vx, vy, -10, duration).join()

    def rotate_to_yaw(self, yaw_degrees):
        """旋转到指定偏航角"""
        self.client.rotateToYawAsync(yaw_degrees).join()

    def capture_camera_image(self, camera_name="front"):
        """捕获相机图像"""
        responses = self.client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene)
        ])
        airsim.write_file(f"{camera_name}_image.png", responses[0].image_data_uint8)
        print(f"Saved {camera_name} camera image.")

    def keyboard_control(self):
        """单线程键盘控制+状态监控"""
        control_map = {
            'w': lambda: self.move_by_velocity(2, 0, 0, 1),  # 前进
            's': lambda: self.move_by_velocity(-2, 0, 0, 1),  # 后退
            'a': lambda: self.move_by_velocity(0, -2, 0, 1),  # 左移
            'd': lambda: self.move_by_velocity(0, 2, 0, 1),  # 右移
            'q': lambda: self.move_by_velocity(0, 0, -1, 1),  # 上升
            'e': lambda: self.move_by_velocity(0, 0, 1, 1),  # 下降
            'r': lambda: self.rotate_to_yaw(90),  # 右转90度
            'f': self.capture_camera_image,  # 拍照
            'esc': lambda: setattr(self, 'stop_mission', True)  # 退出
        }

        print("Keyboard controls: W/S/A/D/Q/E/R/F/ESC")

        while not self.stop_mission:
            try:
                key = keyboard.read_key(suppress=True)
                if key in control_map:
                    control_map[key]()
            except:
                pass

            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            print(
                f"Position: X={pos.x_val:.2f}, Y={pos.y_val:.2f}, Z={-pos.z_val:.2f}m",
                end="\r"
            )
            time.sleep(0.05)


if __name__ == "__main__":
    drone = AdvancedAirSimDroneController()
    drone.takeoff(5)

    try:
        drone.keyboard_control()
    finally:
        drone.land()