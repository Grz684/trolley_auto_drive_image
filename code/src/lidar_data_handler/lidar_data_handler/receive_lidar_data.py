import signal
import sys
import threading
import traceback
import time
import numpy as np
from linear_sensor_msgs.msg import LinearSensorData
from rclpy.executors import SingleThreadedExecutor
import message_filters
import rclpy
import rclpy.signals
import rclpy.time
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

        self.init_draw_count()
        self.sensors_health_count = 0
        # 可用于调试
        self.motor_activate = False
        self.sensor_is_ready = False
        
        self.mode_state_machine = ModeStateMachine()
        self.utils = Utils()

        self.receive_debug = 1
        self.stop_adjust_count = 0
        self.error_state_flag = False

        # 创建定时器，每0.1秒触发一次数据处理
        self.timer_period_sec = 0.1
        self.timer = self.create_timer(self.timer_period_sec, self.set_motor)

        self.init_pid_controller()
        self.init_channels()
        self.init_synchronizer()

        self.check_timer = None
        self.last_sync_time = self.get_clock().now()
        self.check_sensor_duration = 2

        # 退出机制
        # self.future = Future()
        # self.angle2state = {0: "前轮正中", 1: "前轮右转", -1: "前轮左转"}

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
        self.all_sensors_ats = message_filters.ApproximateTimeSynchronizer([self.front_lidar_sub, self.back_lidar_sub, self.left_angle_sub, self.right_angle_sub], 10, self.timer_period_sec)
        self.all_sensors_ats.registerCallback(self.steer_when_drive)
        self.linear_sensors_ats = message_filters.ApproximateTimeSynchronizer([self.left_angle_sub, self.right_angle_sub], 10, self.timer_period_sec)
        self.linear_sensors_ats.registerCallback(self.steer_when_stop)
        self.front_lidar_sub.registerCallback(self.error_state_front_lidar_callback)
        self.back_lidar_sub.registerCallback(self.error_state_back_lidar_callback)
        self.left_angle_sub.registerCallback(self.error_state_left_angle_callback)
        self.right_angle_sub.registerCallback(self.error_state_right_angle_callback)

    def init_pid_controller(self):
        # 初始化pid控制器和数据处理工具
        self.kp = 100
        self.ki = 0
        self.kd = 0
        self.use_pid = 1
        self.pid_controller = PIDController(self.kp, self.ki, self.kd, 1)
        # bound值都是需要实机确认的
        self.mid_lower_bound = 82
        self.mid_upper_bound = 92
        self.mid_bound = 87
        self.left_bound = 23
        self.right_bound = 126

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

    def control_left_oil_cylinder(self, direction):
        if direction == 1:
            # 左油缸收缩
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                print("error")
                # self.future.set_result(None)  # 触发主循环结束
        elif direction == -1:
            # 左油缸拉伸
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

    def set_motor(self):
        if not self.error_state_flag:
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

                if control_f_input_state.value == self.active_state:
                    if self.receive_debug:
                        print("按前进键")
                    mode_exchange = self.mode_state_machine.transition_to_forward()
                elif control_b_input_state.value == self.active_state:
                    if self.receive_debug:
                        print("按后退键")
                    mode_exchange = self.mode_state_machine.transition_to_backward()
                elif control_stop_input_state.value == self.active_state:
                    if self.receive_debug:
                        print("按停止键")
                    mode_exchange = self.mode_state_machine.reset_to_stop()
                    
                # 如果状态转换成功
                if mode_exchange:
                    # 前进或后退模式
                    if control_f_input_state.value == self.active_state or control_b_input_state.value == self.active_state:
                        # 打开雷达罩子
                        # self.open_lidar_mask()
                        # 是否开启电机有待观察
                        if self.receive_debug:
                            print("运动")
                        self.motor_activate = True
                        # 重置pid控制器
                        self.reset_pid_controller()

                    # 停止模式
                    elif control_stop_input_state.value == self.active_state:
                        if self.receive_debug:
                            print("停止")
                        # 重置传感器状态检查器
                        self.check_timer.cancel()
                        # 重置停止调整计数器
                        self.stop_adjust_count = 0

                        # 清除一次过往图像
                        data = {'target': 'clear'}
                        event = PlotUpdateEvent(data)
                        QApplication.postEvent(self.window, event)

                        data = {'target': 'main_program', 'main_program_state': '停止模式'}
                        event = PlotUpdateEvent(data)
                        QApplication.postEvent(self.window, event)

                        self.sensors_health_count = 0
                        self.motor_activate = False
                        self.sensor_is_ready = False

                        self.control_motor(self.mode_state_machine.state)

                        # 关闭雷达罩子时保证左右油缸静止
                        self.control_left_oil_cylinder(0)
                        self.control_right_oil_cylinder(0)
                        # self.close_lidar_mask()

                # 启动前检查传感器数据是否到位
                if self.motor_activate:
                    if self.sensor_is_ready:
                        if self.receive_debug:
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
                        # 为运行过程执行定期传感器检查
                        self.last_sync_time = self.get_clock().now()
                        self.check_timer = self.create_timer(self.timer_period_sec, self.check_sensor_data)
                    else:
                        self.sensors_health_count += 1
                        if self.sensors_health_count > 20:
                            self.error_state_handler()
                            self.error_state_flag = True

    def check_sensor_data(self):
        if self.receive_debug:
            print("check sensor data, last sync time:", self.last_sync_time)
        if self.get_clock().now() - self.last_sync_time > rclpy.time.Duration(seconds=self.check_sensor_duration):
            self.error_state_handler()
            self.error_state_flag = True

    def open_lidar_mask(self):
        if self.receive_debug:
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
        if self.receive_debug:
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

    def steer_when_drive(self, front_lidar_msg, back_lidar_msg, left_angle_msg, right_angle_msg):
        if not self.error_state_flag and self.mode_state_machine.state != 0:
            left_angle_dis = left_angle_msg.data
            right_angle_dis = right_angle_msg.data
            front_lidar_points = np.array(list(pc2.read_points(front_lidar_msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
            back_lidar_points = np.array(list(pc2.read_points(back_lidar_msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
            if front_lidar_points.size == 0 or back_lidar_points.size == 0:
                pass
            else:
                if self.sensor_is_ready is False:
                    self.sensor_is_ready = True

                self.last_sync_time = self.get_clock().now()

                if self.receive_debug:
                    print("-----------------------")
                    print(f"current mode:{self.mode_state_machine.state}")

                front_middle_diff, forward_front_diff, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                    f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(front_lidar_points)
                back_middle_diff, backward_front_diff, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                    b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(back_lidar_points)
                # 每秒绘图
                self.draw_all_count += 1
                if self.draw_all_count == 10:
                    # 更新雷达数据
                    data = {'target': 'all', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                            'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                            'f_average_x_upper_line': f_average_x_upper_line, 'b_t_points': b_t_points, 
                            'b_t_refer_points': b_t_refer_points, 'b_average_y_upper_line': b_average_y_upper_line, 
                            'b_average_y_lower_line': b_average_y_lower_line, 'b_average_x_upper_line': b_average_x_upper_line}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)
                    # 更新拉线传感器数据
                    data = {'target': 'both_oil', 'left_state': str(left_angle_dis),
                            'right_state': str(right_angle_dis)}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)

                    self.draw_all_count = 0
                    
                if self.receive_debug:
                    print(f"front_middle_diff:{float(front_middle_diff)}, back_middle_diff:{-float(back_middle_diff)}")

                if self.use_pid:
                    target_angle = self.pid_controller.pid_handle_drive_state(front_middle_diff, back_middle_diff)
                else:
                    target_angle = self.pid_controller.bang_handle_drive_state(front_middle_diff, back_middle_diff)

                if self.receive_debug:
                    print(f"target_angle:{target_angle}, current_left_angle:{left_angle_dis}, "
                        f"current_right_angle:{right_angle_dis}")

                self.steer_to_target_angle_dis(target_angle, left_angle_dis, right_angle_dis)

    def steer_when_stop(self, left_angle_msg, right_angle_msg):
        if not self.error_state_flag and self.mode_state_machine.state == 0:
            # 按停止键调整轮胎居中
            target_angle = 0
            left_angle_dis = left_angle_msg.data
            right_angle_dis = right_angle_msg.data
            # 更新拉线传感器数据
            data = {'target': 'both_oil', 'left_state': str(left_angle_dis),
                    'right_state': str(right_angle_dis)}
            event = PlotUpdateEvent(data)
            QApplication.postEvent(self.window, event)
            self.steer_to_target_angle_dis(target_angle, left_angle_dis, right_angle_dis)

    def error_state_left_angle_callback(self, msg):
        if self.error_state_flag:
            self.draw_left_angle_count += 1
            if self.draw_left_angle_count == 10:
                left_angle_dis = msg.data
                data = {'target': 'left_oil', 'state': str(left_angle_dis)}
                event = PlotUpdateEvent(data)
                QApplication.postEvent(self.window, event)
                self.draw_left_angle_count = 0

    def error_state_right_angle_callback(self, msg):
        if self.error_state_flag:
            self.draw_right_angle_count += 1
            if self.draw_right_angle_count == 10:
                right_angle_dis = msg.data
                data = {'target': 'right_oil', 'state': str(right_angle_dis)}
                event = PlotUpdateEvent(data)
                QApplication.postEvent(self.window, event)
                self.draw_right_angle_count = 0

    def error_state_front_lidar_callback(self, msg):
        if self.error_state_flag:
            self.draw_front_lidar_count += 1
            if self.draw_front_lidar_count == 10:
                front_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
                if front_lidar_points.size == 0:
                    pass
                else:
                    _, _, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                            f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(front_lidar_points)
                    data = {'target': 'front', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                            'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                            'f_average_x_upper_line': f_average_x_upper_line}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)
                self.draw_front_lidar_count = 0

    def error_state_back_lidar_callback(self, msg):
        if self.error_state_flag:
            self.draw_back_lidar_count += 1
            if self.draw_back_lidar_count == 10:
                back_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
                if back_lidar_points.size == 0:
                    pass
                else:
                    _, _, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                            b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(back_lidar_points)
                    data = {'target': 'back', 'b_t_points': b_t_points, 'b_t_refer_points': b_t_refer_points,
                            'b_average_y_upper_line': b_average_y_upper_line, 'b_average_y_lower_line': b_average_y_lower_line,
                            'b_average_x_upper_line': b_average_x_upper_line}
                    event = PlotUpdateEvent(data)
                    QApplication.postEvent(self.window, event)
                self.draw_back_lidar_count = 0

    def steer_to_target_angle_dis(self, target_angle, left_angle_dis, right_angle_dis):
        left_cylinder_target_direction = self.get_target_direction(target_angle, left_angle_dis)
        right_cylinder_target_direction = self.get_target_direction(target_angle, right_angle_dis)
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

    def get_target_direction(self, target_angle, current_angle_dis):
        # 先把current_angle（拉线长度）映射到target_angle的量程（-50~50）
        # current_angle_dis较小时，轮胎朝左；current_angle_dis较大时，轮胎朝右
        if self.mid_lower_bound <= current_angle_dis <= self.mid_upper_bound:
            current_angle = 0
        elif current_angle_dis < self.mid_lower_bound:
            current_angle = (self.mid_bound - current_angle_dis)/(self.mid_bound-self.left_bound)*50
        else:
            current_angle = (self.mid_bound - current_angle_dis)/(self.right_bound-self.mid_bound)*50

        if current_angle == target_angle:
            return int(0)
        elif current_angle < target_angle:
            # 当前过右偏了，往左打方向（current和target都可理解为轮胎角度）
            return int(1)
        else:
            return int(-1)

        # direction为1是收缩，为-1是拉伸
        # if target_angle == 0:
        #     if current_angle == 0:
        #         return int(0)
        #     elif current_angle == -1:
        #         # 向右打方向
        #         return int(-1)
        #     elif current_angle == 1:
        #         # 向左打方向
        #         return int(1)
        # elif target_angle == 1:
        #     # 目标向左，油缸收缩
        #     return int(1)
        # elif target_angle == -1:
        #     # 目标向右，油缸拉伸
        #     return int(-1)

    def error_state_handler(self):
        print("主程序停止")
        # 清除一次过往图像
        data = {'target': 'clear'}
        event = PlotUpdateEvent(data)
        QApplication.postEvent(self.window, event)
        # 停止计时器
        self.timer.cancel()
        self.check_timer.cancel()
        # 停止一切驱动输出
        for i in range(10):
            IO_WritePin(self.sn, i, self.inactive_state)

        data = {'target': 'main_program', 'main_program_state': '运行故障'}
        event = PlotUpdateEvent(data)
        QApplication.postEvent(self.window, event)

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
        finally:
            print("停止所有驱动输出!!!")
            # sn = 231202311
            # inactive_state = 1
            for i in range(16):
                ret = IO_WritePin(lidar_data_handler.sn, i, lidar_data_handler.inactive_state)
                if ret < 0:
                    print(f"Failed to close IO{i}")
            
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
