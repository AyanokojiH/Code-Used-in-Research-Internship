import os
import sys
import sumolib
import traci
import yaml
import mysql.connector
from mysql.connector import Error

class SUMOSimulator:
    def __init__(self, sumo_config_path):
        """
        初始化 SUMO 模拟器。
        :param sumo_config_path: SUMO 配置文件路径。
        """
        self.sumo_config_path = sumo_config_path
        self.vehicle_positions = {}

    def start_simulation(self):
        """
        启动 SUMO 模拟并收集车辆坐标数据。
        """
        if 'SUMO_HOME' not in os.environ:
            sys.exit("Please add SUMO_HOME to path")
        print("Successfully entered SUMO...")

        sumo_binary = sumolib.checkBinary('sumo')
        sumo_cmd = [sumo_binary, "-c", self.sumo_config_path]
        traci.start(sumo_cmd)

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_step = traci.simulation.getTime()
            vehicles = traci.vehicle.getIDList()

            for vehicle_id in vehicles:
                if vehicle_id not in self.vehicle_positions:
                    self.vehicle_positions[vehicle_id] = []
                position = traci.vehicle.getPosition(vehicle_id)
                self.vehicle_positions[vehicle_id].append((current_step, position[0], position[1]))

        traci.close()
        print("SUMO 模拟完成，车辆坐标数据已收集。")

    def get_vehicle_positions(self):
        """
        获取收集到的车辆坐标数据。
        :return: 车辆坐标数据字典。
        """
        return self.vehicle_positions


class MySQLDatabase:
    def __init__(self, config_path):
        """
        初始化 MySQL 数据库连接。
        :param config_path: 数据库配置文件路径。
        """
        self.config = self.load_config(config_path)
        self.connection = None

    @staticmethod
    def load_config(filename):
        """
        加载 YAML 配置文件。
        :param filename: 配置文件路径。
        :return: 配置字典。
        """
        with open(filename, 'r') as file:
            return yaml.safe_load(file)

    def connect(self):
        """
        连接到 MySQL 数据库。
        """
        try:
            self.connection = mysql.connector.connect(**self.config)
            print("Connected to MySQL server")
        except Error as e:
            print(f"数据库连接失败：{e}")

    def save_vehicle_positions(self, vehicle_positions):
        """
        将车辆坐标数据保存到 MySQL 数据库。
        :param vehicle_positions: 车辆坐标数据字典。
        """
        if not self.connection:
            print("未连接到数据库，请先调用 connect() 方法。")
            return

        cursor = self.connection.cursor()

        for vehicle_id, positions in vehicle_positions.items():
            table_name = f"vehicle_{vehicle_id.replace('.', '_')}"
            cursor.execute(f"""
                CREATE TABLE IF NOT EXISTS {table_name} (
                    step INTEGER,
                    x REAL,
                    y REAL
                )
            """)
            print(f"为车辆 {vehicle_id} 创建了表：{table_name}")

            cursor.executemany(f"INSERT INTO {table_name} (step, x, y) VALUES (%s, %s, %s)", positions)
            print(f"车辆 {vehicle_id} 的数据已插入到表 {table_name}")

        self.connection.commit()
        print("所有数据已成功保存到 MySQL 数据库")

    def close(self):
        """
        关闭数据库连接。
        """
        if self.connection and self.connection.is_connected():
            self.connection.close()
            print("MySQL 数据库连接已关闭")


def main():
    sumo_config_path = "D:\\SUMO\\crossing.sumocfg"  # 我的用例
    db_config_path = "config_timer.yaml"

    simulator = SUMOSimulator(sumo_config_path)
    simulator.start_simulation()
    vehicle_positions = simulator.get_vehicle_positions()

    db = MySQLDatabase(db_config_path)
    db.connect()
    db.save_vehicle_positions(vehicle_positions)
    db.close()


if __name__ == "__main__":
    main()