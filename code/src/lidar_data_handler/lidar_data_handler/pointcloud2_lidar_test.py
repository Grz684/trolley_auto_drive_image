import rclpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
import matplotlib.pyplot as plt
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from handle_lidar_data_utils import Utils
from rclpy.qos import qos_profile_sensor_data


class LidarDataHandler(Node):
    def __init__(self):
        super().__init__('lidar_data_handler')

        # 接收雷达数据
        self.lidar_sub1 = self.create_subscription(
            PointCloud2,
            '/back_lidar/cloud',
            self.lidar_callback1,
            qos_profile_sensor_data)

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

    def lidar_callback1(self, msg):
        points = []
        for point in pc2.read_points(msg, field_names=["x", "y"], skip_nans=True):
            x, y = point
            points.append((x, y))
        # front_middle_diff, forward_front_diff = self.utils.get_diff(points)
        # print(f'front_middle_diff:{front_middle_diff}, forward_front_diff:{forward_front_diff}')

        self.draw_count += 1
        if self.draw_count == 10:
            self.plot_point_cloud(points)
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
