import signal
import sys
import threading
import traceback
import time
import numpy as np

import rclpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from std_msgs.msg import Int8
from .handle_lidar_data_utils import Utils
from .pid_controller import PIDController
from .mode_state_machine import ModeStateMachine
from rclpy.qos import qos_profile_sensor_data
from .public.librockmong import *
from .public.usb_device import *
from .public.gpio import *
from .control_ui import *


class LidarDataHandler(Node):

    def __init__(self, gui):
        super().__init__('lidar_data_handler')
        self.window = gui
        self.draw_count = 0
        self.sensors_health_count = 0
        self.motor_activate = False
        
        self.mode_state_machine = ModeStateMachine()
        self.utils = Utils()

        self.receive_debug = 0
        self.stop_adjust_count = 0
        self.stop_flag = False

        self.init_pid_controller()
        self.init_channels()
        self.init_display_subscription()

        # 创建定时器，每0.1秒触发一次数据处理并发布油缸运动方向
        self.timer_period_sec = 0.1
        self.timer = self.create_timer(self.timer_period_sec, self.process_and_drive)

        # 退出机制
        # self.future = Future()

    def init_pid_controller(self):
        # 初始化pid控制器和数据处理工具
        self.kp = 8
        self.ki = 0
        self.kd = 0
        self.use_pid = 0
        self.pid_controller = PIDController(self.kp, self.ki, self.kd, 1)

    def init_display_subscription(self):
        self.front_latest_lidar_data = None
        self.back_latest_lidar_data = None
        self.left_angle = None
        self.right_angle = None
        # lidar1为前雷达，lidar2为后雷达，测距器1在左，测距器2在右
        self.lidar_sub1 = self.create_subscription(
            PointCloud2,
            '/front_lidar/cloud',
            self.lidar_callback1,
            qos_profile_sensor_data)
        self.lidar_sub2 = self.create_subscription(
            PointCloud2,
            '/back_lidar/cloud',
            self.lidar_callback2,
            qos_profile_sensor_data)

        # 初始化当前左车桥转向角和右车桥转向角订阅者，以距离代替角度
        self.left_angle_sub = self.create_subscription(
            Int8,
            'left_angle',
            self.left_angle_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.right_angle_sub = self.create_subscription(
            Int8,
            'right_angle',
            self.right_angle_callback,
            qos_profile=qos_profile_sensor_data
        )

    def init_channels(self):
        # --------------digital_switch-------------
        self.active_state = 0
        self.inactive_state = 1
        self.sn = 231202311
        # 控制电机正转/反转
        self.motor_f_output_channel = 0
        self.motor_b_output_channel = 1
        # ll为左油缸收缩，lr为左油缸伸长，rl为右油缸收缩，rr为右油缸伸长
        self.cylinder_ll_output_channel = 2
        self.cylinder_lr_output_channel = 3
        self.cylinder_rl_output_channel = 4
        self.cylinder_rr_output_channel = 5
        # 控制线前进/后退输入及电机前转/反转输出
        self.control_f_input_channel = 0
        self.control_b_input_channel = 1
        self.control_stop_input_channel = 2
        # 雷达罩子
        self.front_lidar_mask_open_channel = 6
        self.front_lidar_mask_close_channel = 7
        self.back_lidar_mask_open_channel = 8
        self.back_lidar_mask_close_channel = 9

        # Scan device
        serial_numbers = (c_int * 20)()
        ret = UsbDevice_Scan(byref(serial_numbers))
        if 0 > ret:
            print("Error: %d" % ret)
            exit()
        elif ret == 0:
            print("No device!")
            exit()
        elif self.sn not in list(serial_numbers):
            print("No target device")
            exit()

    def reset_pid_controller(self):
        if self.mode_state_machine.state == 1 or self.mode_state_machine.state == -1:
            self.pid_controller = PIDController(self.kp, self.ki, self.kd, self.mode_state_machine.state)

    def left_angle_callback(self, msg):
        self.left_angle = msg.data

    def right_angle_callback(self, msg):
        self.right_angle = msg.data

    def lidar_callback1(self, msg):
        points_array = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
        self.front_latest_lidar_data = points_array

        # points = msg.points
        # self.front_latest_lidar_data = [(point.x, point.y) for point in points]

    def lidar_callback2(self, msg):
        points_array = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
        self.back_latest_lidar_data = points_array

    def control_left_oil_cylinder(self, direction):
        if direction == 1:
            # 油缸左伸
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        elif direction == -1:
            # 油缸右伸
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        else:
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束

    def control_right_oil_cylinder(self, direction):
        if direction == 1:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        elif direction == -1:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        else:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束

    def control_motor(self, mode):
        if mode == 1:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        elif mode == -1:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        elif mode == 0:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
            print("Reset to Stop")

    def set_mode(self):
        control_b_input_state = c_int()
        control_f_input_state = c_int()
        control_stop_input_state = c_int()
        ret1 = IO_ReadPin(self.sn, self.control_b_input_channel, byref(control_b_input_state))
        ret2 = IO_ReadPin(self.sn, self.control_f_input_channel, byref(control_f_input_state))
        ret3 = IO_ReadPin(self.sn, self.control_stop_input_channel, byref(control_stop_input_state))
        if 0 > ret1 or 0 > ret2 or 0 > ret3:
            print("error input")
            # self.future.set_result(None)  # 触发主循环结束
        else:
            mode_exchange = False

            obtain_all_sensors_data_flag = (self.front_latest_lidar_data is not None
                                            and self.back_latest_lidar_data is not None
                                            and self.left_angle is not None
                                            and self.right_angle is not None)

            if control_f_input_state.value == self.active_state:
                mode_exchange = self.mode_state_machine.transition_to_forward()
            elif control_b_input_state.value == self.active_state:
                mode_exchange = self.mode_state_machine.transition_to_backward()
            elif control_stop_input_state.value == self.active_state:
                mode_exchange = self.mode_state_machine.reset_to_stop()
                
            if mode_exchange:
                if control_f_input_state.value == self.active_state or control_b_input_state.value == self.active_state:
                    self.open_lidar_mask()
                    self.motor_activate = True
                    self.reset_pid_controller()

                elif control_stop_input_state.value == self.active_state:
                    self.stop_adjust_count = 0
                    data = {'target': 'main_program', 'main_program_state': '停止模式'}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)
                    self.sensors_health_count = 0
                    self.motor_activate = False
                    self.control_motor(self.mode_state_machine.state)

                    # 关闭雷达罩子时保证左右油缸静止
                    self.control_left_oil_cylinder(0)
                    self.control_right_oil_cylinder(0)
                    self.close_lidar_mask()

            # 启动前检查传感器数据是否到位
            if self.motor_activate:
                if obtain_all_sensors_data_flag:
                    print("传感器数据正常，启动电机")
                    if self.mode_state_machine.state == 1:
                        data = {'target': 'main_program', 'main_program_state': '前进模式'}
                    elif self.mode_state_machine.state == -1:
                        data = {'target': 'main_program', 'main_program_state': '后退模式'}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)
                    self.motor_activate = False
                    self.sensors_health_count = 0
                    self.control_motor(self.mode_state_machine.state)
                else:
                    self.sensors_health_count += 1
                    if self.sensors_health_count > 20:
                        self.stop()
                        self.stop_flag = True

            # 运行过程中检查传感器数据是否到位
            if self.mode_state_machine.state != 0 and self.motor_activate is False:
                if obtain_all_sensors_data_flag:
                    self.sensors_health_count = 0
                else:
                    self.sensors_health_count += 1
                    if self.sensors_health_count > 20:
                        self.stop()
                        self.stop_flag = True

                    

    def open_lidar_mask(self):
        self.get_logger().info("开启雷达罩子，给电3s")
        ret1 = IO_WritePin(self.sn, self.front_lidar_mask_open_channel, self.active_state)
        ret2 = IO_WritePin(self.sn, self.front_lidar_mask_close_channel, self.inactive_state)
        ret3 = IO_WritePin(self.sn, self.back_lidar_mask_open_channel, self.active_state)
        ret4 = IO_WritePin(self.sn, self.back_lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2 or 0 > ret3 or 0 > ret4:
            print("error")
        time.sleep(3)
        ret1 = IO_WritePin(self.sn, self.front_lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.front_lidar_mask_close_channel, self.inactive_state)
        ret3 = IO_WritePin(self.sn, self.back_lidar_mask_open_channel, self.inactive_state)
        ret4 = IO_WritePin(self.sn, self.back_lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2 or 0 > ret3 or 0 > ret4:
            print("error")

    def close_lidar_mask(self):
        self.get_logger().info("关闭雷达罩子，给电3s")
        ret1 = IO_WritePin(self.sn, self.front_lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.front_lidar_mask_close_channel, self.active_state)
        ret3 = IO_WritePin(self.sn, self.back_lidar_mask_open_channel, self.inactive_state)
        ret4 = IO_WritePin(self.sn, self.back_lidar_mask_close_channel, self.active_state)
        if 0 > ret1 or 0 > ret2 or 0 > ret3 or 0 > ret4:
            print("error")
        time.sleep(3)
        ret1 = IO_WritePin(self.sn, self.front_lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.front_lidar_mask_close_channel, self.inactive_state)
        ret3 = IO_WritePin(self.sn, self.back_lidar_mask_open_channel, self.inactive_state)
        ret4 = IO_WritePin(self.sn, self.back_lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2 or 0 > ret3 or 0 > ret4:
            print("error")

    def display_lidar_data(self):
        front_middle_diff = None
        back_middle_diff = None
        # 检查是否有新的激光数据
        if self.front_latest_lidar_data is not None:
            front_middle_diff, forward_front_diff, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(self.front_latest_lidar_data)
        if self.back_latest_lidar_data is not None:
            back_middle_diff, backward_front_diff, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(self.back_latest_lidar_data)
            
        if self.front_latest_lidar_data is not None and self.back_latest_lidar_data is not None:
            # 雷达数据可视化
            data = {'target': 'all', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                        'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                        'f_average_x_upper_line': f_average_x_upper_line, 'b_t_points': b_t_points, 
                        'b_t_refer_points': b_t_refer_points, 'b_average_y_upper_line': b_average_y_upper_line, 
                        'b_average_y_lower_line': b_average_y_lower_line, 'b_average_x_upper_line': b_average_x_upper_line}
        elif self.front_latest_lidar_data is not None and self.back_latest_lidar_data is None:
            # 雷达数据可视化
            front_middle_diff, forward_front_diff, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(self.front_latest_lidar_data)
            data = {'target': 'front', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                        'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                        'f_average_x_upper_line': f_average_x_upper_line}
        elif self.front_latest_lidar_data is None and self.back_latest_lidar_data is not None:
            # 雷达数据可视化
            back_middle_diff, backward_front_diff, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(self.back_latest_lidar_data)
            data = {'target': 'back', 'b_t_points': b_t_points, 'b_t_refer_points': b_t_refer_points,
                        'b_average_y_upper_line': b_average_y_upper_line, 'b_average_y_lower_line': b_average_y_lower_line,
                        'b_average_x_upper_line': b_average_x_upper_line}
        else:
            data = {'target': 'none'}

        event = PlotUpdateEvent(data)
        QApplication.postEvent(self.window, event)

        return front_middle_diff, back_middle_diff
    
    def display_linear_data(self):
        angle2state = {0: "前轮正中", 1: "前轮右转", -1: "前轮左转"}
        if self.left_angle is not None and self.right_angle is not None:
            data = {'target': 'oil_state', 'left_oil_state': angle2state[self.left_angle],
                    'right_oil_state': angle2state[self.right_angle]}
        elif self.left_angle is None and self.right_angle is not None:
            data = {'target': 'oil_state', 'left_oil_state': "未知", 'right_oil_state': angle2state[self.right_angle]}
        elif self.left_angle is not None and self.right_angle is None:
            data = {'target': 'oil_state', 'left_oil_state': angle2state[self.left_angle], 'right_oil_state': "未知"}
        else:
            data = {'target': 'oil_state', 'left_oil_state': "未知", 'right_oil_state': "未知"}
        
        event = PlotUpdateEvent(data)
        QApplication.postEvent(self.window, event)

    def process_and_drive(self):
        self.draw_count += 1
        get_draw_data = False
        if self.draw_count == 10:
            front_middle_diff, back_middle_diff = self.display_lidar_data()
            self.display_linear_data()
            get_draw_data = True
            self.draw_count = 0

        if not self.stop_flag:
            self.set_mode()
            if self.receive_debug:
                print("-----------------------")
                print(f"current mode:{self.mode_state_machine.state}")
            # 检查是否有新的激光数据
            if self.front_latest_lidar_data is not None and self.back_latest_lidar_data is not None:
                if not get_draw_data:
                    front_middle_diff, *_ = self.utils.get_diff(self.front_latest_lidar_data)
                    back_middle_diff, *_ = self.utils.get_diff(self.back_latest_lidar_data)

                if self.receive_debug:
                    print(f"front_middle_diff:{float(front_middle_diff)}, back_middle_diff:{-float(back_middle_diff)}")

                if self.use_pid:
                    target_angle = self.pid_controller.pid_handle_drive_state(front_middle_diff, back_middle_diff)
                else:
                    target_angle = self.pid_controller.bang_handle_drive_state(front_middle_diff, back_middle_diff)

                if self.mode_state_machine.state == 0:
                    target_angle = int(0)

                if self.receive_debug:
                    print(f"target_angle:{target_angle}, current_left_angle:{self.left_angle}, "
                          f"current_right_angle:{self.right_angle}")

                self.steer_to_target_angle_dis(target_angle)

    def steer_to_target_angle_dis(self, target_angle):
        if self.left_angle is not None and self.right_angle is not None:
            left_cylinder_target_direction = self.get_target_direction(target_angle, self.left_angle)
            right_cylinder_target_direction = self.get_target_direction(target_angle, self.right_angle)
            if self.mode_state_machine.state == 0:
                if self.stop_adjust_count == 20:
                    left_cylinder_target_direction = 0
                    right_cylinder_target_direction = 0
                else:
                    self.stop_adjust_count = self.stop_adjust_count + 1

            if self.receive_debug:
                print(f'Published left oil cylinder direction: {left_cylinder_target_direction},'
                    f'Published right oil cylinder direction: {right_cylinder_target_direction}')

            self.control_left_oil_cylinder(left_cylinder_target_direction)
            self.control_right_oil_cylinder(right_cylinder_target_direction)
            # 清空数据，避免重复处理
            self.right_angle = None
            self.left_angle = None
            self.front_latest_lidar_data = None
            self.back_latest_lidar_data = None

    @staticmethod
    def get_target_direction(target_angle, current_angle):
        if target_angle == 0:
            if current_angle == 0:
                return int(0)
            elif current_angle == -1:
                # 向左打方向，向右顶油缸
                return int(-1)
            elif current_angle == 1:
                # 向右打方向，向左顶油缸
                return int(1)
        elif target_angle == 1:
            # 目标向左，向左打方向，向右顶油缸
            return int(1)
        elif target_angle == -1:
            # 目标向右，向右打方向，向左顶油缸
            return int(-1)

    def stop(self):
        print("主程序停止")
        # 停止一切驱动输出
        for i in range(6):
            IO_WritePin(self.sn, i, self.inactive_state)

        data = {'target': 'main_program', 'main_program_state': '运行故障'}
        event = PlotUpdateEvent(data)
        QApplication.postEvent(self.window, event)

def main():
    # --------------display_screen-------------
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # Create and start the ROS2 thread
    # 初始化ROS节点
    rclpy.init()
    # 创建节点
    lidar_data_handler = LidarDataHandler(window)

    def run_ros_node():
        try:
            # 保持节点活动
            # rclpy.spin_until_future_complete(lidar_data_handler, lidar_data_handler.future)
            rclpy.spin(lidar_data_handler)
        except Exception as e:
            lidar_data_handler.stop()
            lidar_data_handler.destroy_node()
            rclpy.shutdown()
            app.quit()
            print(f"Caught an exception: {e}")
            traceback.print_exc()
            
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()

    # 信号处理函数,参数是必须的！
    def signal_handler(signum, frame):
        print("停止所有驱动输出!!!")
        # sn = 231202311
        # inactive_state = 1
        for i in range(16):
            ret = IO_WritePin(lidar_data_handler.sn, i, lidar_data_handler.inactive_state)
            if ret < 0:
                print(f"Failed to close IO{i}")
        # 结束qt事件循环
        app.quit()
        # 解除节点资源
        lidar_data_handler.destroy_node()
        rclpy.shutdown()

    # 注册 SIGINT 和 SIGTERM 信号处理器【必要】
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    # .exec_()为qt事件循环函数，在.quit方法后退出
    exit_code = app.exec_()
    print("退出qt事件循环")
    ros_thread.join()
    print("终止ros2节点运行")
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
