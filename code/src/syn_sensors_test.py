import message_filters
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from linear_sensor_msgs.msg import LinearSensorData
import rclpy
from rclpy.node import Node

class SynSensorsTest(Node):
    def __init__(self):
        super().__init__('syn_sensors_test')

        self.lidar_sub1 = message_filters.Subscriber(
          self,
          PointCloud2,
          '/front_lidar/cloud',
          qos_profile=qos_profile_sensor_data
        )

        self.lidar_sub2 = message_filters.Subscriber(
          self,
          PointCloud2,
          '/back_lidar/cloud',
          qos_profile=qos_profile_sensor_data
        )

        # 初始化当前左车桥转向角和右车桥转向角订阅者，以距离代替角度
        self.left_angle_sub = message_filters.Subscriber(
            self,
            LinearSensorData,
            'left_angle',
            qos_profile=qos_profile_sensor_data
        )
        self.right_angle_sub = message_filters.Subscriber(
            self,
            LinearSensorData,
            'right_angle',
            qos_profile=qos_profile_sensor_data
        )
        ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub1, self.lidar_sub2, self.left_angle_sub, self.right_angle_sub], 10, 0.1)
        # ts = message_filters.TimeSynchronizer([self.lidar_sub1, self.lidar_sub2, self.left_angle_sub, self.right_angle_sub], 10)
        ts.registerCallback(self.callback_function)

  # 同步订阅者
    def callback_function(self, front_lidar_data, back_lidar_data, left_angle_data, right_angle_data):
        # 处理同步的数据
        print(front_lidar_data.header.frame_id, back_lidar_data.header.frame_id, left_angle_data.data, right_angle_data.data)


def main():
    # 初始化ROS节点
    rclpy.init()
    # 创建节点
    synSensorsTest = SynSensorsTest()

    
    # 保持节点活动
    # rclpy.spin_until_future_complete(lidar_data_handler, lidar_data_handler.future)
    rclpy.spin(synSensorsTest)

    synSensorsTest.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()