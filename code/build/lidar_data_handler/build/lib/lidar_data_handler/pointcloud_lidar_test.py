import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud, LaserScan
from handle_lidar_data_utils import Utils
from rclpy.node import Node
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import sys

sys.path.append("/opt/ros/humble/lib/python3.10/site-packages")
sys.path.append("/opt/ros/humble/local/lib/python3.10/dist-packages")


class LidarDataHandler(Node):
    def __init__(self):
        super().__init__('lidar_data_handler')

        # 接收雷达数据
        self.lidar_sub1 = self.create_subscription(
            PointCloud,
            '/ros_node/cloud',
            self.lidar_callback,
            qos_profile_sensor_data)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.utils = Utils()
        self.draw_count = 0

    @staticmethod
    def plot_point_cloud(points):
        # 分离 x 和 y 坐标
        x_coords, y_coords = zip(*points)

        # 创建一个新的图
        plt.figure()

        # 绘制点集
        plt.scatter(x_coords, y_coords)

        # 显示图
        # plt.savefig('filename.png')
        plt.show()

    def lidar_callback(self, msg):
        # 获取点云数据
        points = msg.points

        # 将点云数据转换为[(x1, y1), (x2, y2), ...]的格式
        xy_coordinates = [(point.x, point.y) for point in points]

        front_middle_diff, forward_front_diff = self.utils.get_diff(xy_coordinates)
        print(f'front_middle_diff:{front_middle_diff}, forward_front_diff:{forward_front_diff}')

        # self.draw_count += 1
        # if self.draw_count == 10:
        #     # self.plot_point_cloud(xy_coordinates)
        #     self.draw_count = 0

    def scan_callback(self, msg):
        # 假设 msg.angle_min, msg.angle_max, msg.angle_increment 是从 LaserScan 消息获取的
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        distances = np.array(msg.ranges)

        # 修正的部分: 计算角度数组
        num_points = len(distances)
        angles = np.linspace(angle_min, angle_max, num_points)

        # Convert to Cartesian coordinates for plotting
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)

        self.draw_count += 1
        if self.draw_count == 10:
            plt.polar(angles, distances, 'b.')
            plt.show()
            self.draw_count = 0


if __name__ == '__main__':
    # 初始化ROS节点
    rclpy.init()

    # 创建节点
    lidar_data_handler = LidarDataHandler()

    # 保持节点活动
    rclpy.spin(lidar_data_handler)

    # 关闭节点
    lidar_data_handler.destroy_node()
    rclpy.shutdown()
