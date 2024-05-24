import signal
import sys
import threading
import traceback
import numpy as np
import message_filters
import rclpy
import rclpy.signals
import rclpy.time
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from handle_lidar_data_utils import Utils
from rclpy.qos import qos_profile_sensor_data
from control_ui import *


class LidarDataHandler(Node):

    def __init__(self, gui):
        super().__init__('lidar_data_handler')
        self.window = gui
        self.utils = Utils()
        self.init_draw_count()
        self.init_synchronizer()
        

    def init_draw_count(self):
        self.draw_all_count = 0
        self.draw_back_lidar_count = 0
        self.draw_front_lidar_count = 0
        self.draw_left_angle_count = 0
        self.draw_right_angle_count = 0

    def init_synchronizer(self):
        self.front_lidar_sub = message_filters.Subscriber(
          self,
          PointCloud2,
          '/front_lidar/cloud',
          qos_profile=qos_profile_sensor_data
        )

        self.back_lidar_sub = message_filters.Subscriber(
          self,
          PointCloud2,
          '/back_lidar/cloud',
          qos_profile=qos_profile_sensor_data
        )

        self.front_lidar_sub.registerCallback(self.error_state_front_lidar_callback)
        self.back_lidar_sub.registerCallback(self.error_state_back_lidar_callback)

    def error_state_front_lidar_callback(self, msg):
        self.draw_front_lidar_count += 1
        if self.draw_front_lidar_count == 10:
            front_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
            if front_lidar_points.size == 0:
                pass
            else:
                front_middle_diff, _, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                        f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(front_lidar_points)
                
                print(f"front_middle_diff:{float(front_middle_diff)}")

                data = {'target': 'front', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                        'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                        'f_average_x_upper_line': f_average_x_upper_line}
                event = PlotUpdateEvent(data)
                QApplication.postEvent(self.window, event)
            self.draw_front_lidar_count = 0

    def error_state_back_lidar_callback(self, msg):
        self.draw_back_lidar_count += 1
        if self.draw_back_lidar_count == 10:
            back_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
            if back_lidar_points.size == 0:
                pass
            else:
                back_middle_diff, _, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                        b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(back_lidar_points)
                
                print(f"back_middle_diff:{-float(back_middle_diff)}")

                data = {'target': 'back', 'b_t_points': b_t_points, 'b_t_refer_points': b_t_refer_points,
                        'b_average_y_upper_line': b_average_y_upper_line, 'b_average_y_lower_line': b_average_y_lower_line,
                        'b_average_x_upper_line': b_average_x_upper_line}
                event = PlotUpdateEvent(data)
                QApplication.postEvent(self.window, event)
            self.draw_back_lidar_count = 0

def main():
    # --------------display_screen-------------
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    rclpy.init()
    lidar_data_handler = LidarDataHandler(window)

    def run_ros_node():
        try:
            # 初始化ROS节点
            rclpy.spin(lidar_data_handler)
        except Exception as e:
            print(f"Caught an exception: {e}")
            traceback.print_exc()
            app.quit()
            # 解除节点资源
            lidar_data_handler.destroy_node()
            rclpy.shutdown()
        finally:
            print("停止")
            
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()

    # 信号处理函数,参数是必须的！
    def signal_handler(signum, frame):
        # 结束qt事件循环
        app.quit()
        # 解除节点资源
        lidar_data_handler.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    timer = QTimer()
    timer.start(100)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 100 ms.

    # .exec_()为qt事件循环函数，在.quit方法后退出
    exit_code = app.exec_()
    print("退出qt事件循环")
    ros_thread.join()
    print("终止ros2节点运行")
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
