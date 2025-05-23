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
from std_msgs.msg import Int8, Empty
from .handle_lidar_data_utils import Utils
from .pid_controller import PIDController
from .mode_state_machine import ModeStateMachine
from rclpy.qos import qos_profile_sensor_data
from .public.librockmong import *
from .public.usb_device import *
from .public.gpio import *
from .control_ui import *
from PyQt5.QtCore import QThread, pyqtSignal
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DIO_Error(Exception):
    pass

class LidarDataHandlerThread(QThread):
    display_error = pyqtSignal(str)
    plotUpdateSignal = pyqtSignal(dict)
    initializationError = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._is_stopped = False
        self.lidar_data_handler = None
        self.sn = 231202311
        self.run_indicator_channel = 15
        self.fault_indicator_channel = 14
        self.active_state = 0
        self.inactive_state = 1

    def initialize(self):
        try:
            rclpy.init()
            self.lidar_data_handler = LidarDataHandler(self)
        except DIO_Error as e:
            logger.error("Caught a DIO exception: %s", e)
            self.initializationError.emit("dio")

    def handle_initialization_error(self):
        data = {'target': 'clear'}
        self.plotUpdateSignal.emit(data)
        self.display_error.emit("dio")
        data = {'target': 'main_program', 'main_program_state': '运行故障'}
        self.plotUpdateSignal.emit(data)

    def run(self):
        try:
            ret = IO_WritePin(self.sn, self.run_indicator_channel, self.active_state)
            if 0 > ret:
                raise DIO_Error("运行指示灯初始化失败")
            rclpy.spin(self.lidar_data_handler)
        except DIO_Error as e:
            logger.error("Caught an dio exception: %s", e)
            data = {'target': 'clear'}
            self.plotUpdateSignal.emit(data)
            self.display_error.emit("dio")
            # 显示运行故障
            data = {'target': 'main_program', 'main_program_state': '运行故障'}
            self.plotUpdateSignal.emit(data)
        except Exception as e:
            # 在需要输出的地方使用
            logger.error("Caught an exception: %s\n%s", e, traceback.format_exc())
            data = {'target': 'clear'}
            self.plotUpdateSignal.emit(data)
            self.display_error.emit("program")
            # 显示运行故障
            data = {'target': 'main_program', 'main_program_state': '程序报错'}
            self.plotUpdateSignal.emit(data)

            ret1 = IO_WritePin(self.sn, self.run_indicator_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.fault_indicator_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                logger.error("错误指示灯初始化失败")

        finally:
            logger.info("停止所有驱动输出!!!")
            for i in range(16):
                ret = IO_WritePin(self.lidar_data_handler.sn, i, self.lidar_data_handler.inactive_state)
                if ret < 0:
                    logger.error("Failed to close IO")
                    break

    def stop(self):
        if not self._is_stopped:
            self._is_stopped = True
            if self.lidar_data_handler:
                self.lidar_data_handler.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.quit()
            self.wait()

class LidarDataHandler(Node):
    def __init__(self, thread):
        super().__init__('lidar_data_handler')
        self.config_file = 'settings.json'
        self.thread = thread

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
        self.reset_publisher = self.create_publisher(Empty, 'reset_encoder', 10)

        self.init_pid_controller()
        self.init_channels()
        self.init_synchronizer()

        self.drive_last_sync_time = time.time()
        self.stop_last_sync_time = time.time()
        self.sensor_when_normal_timeout = 2.0

        current_time = time.time()
        self.sensor_last_update = {
            'left_angle': current_time,
            'right_angle': current_time,
            'front_lidar': current_time,
            'back_lidar': current_time
        }
        self.sensor_when_error_timeout = 2.0  # 错误或停止状态下2秒超时
        
        self.check_sensor_when_error_timer = self.create_timer(1.0, self.check_sensor_timeout)

        self.check_sensor_when_drive_timer = self.create_timer(0.5, self.check_sensor_data_when_drive)
        self.check_sensor_when_stop_timer = self.create_timer(0.5, self.check_sensor_data_when_stop)

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
        self.all_sensors_ats = message_filters.ApproximateTimeSynchronizer([self.front_lidar_sub, self.back_lidar_sub, self.left_angle_sub, self.right_angle_sub], 10, self.timer_period_sec*2)
        self.all_sensors_ats.registerCallback(self.steer_when_drive)
        self.linear_sensors_ats = message_filters.ApproximateTimeSynchronizer([self.left_angle_sub, self.right_angle_sub], 10, self.timer_period_sec*2)
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
        self.angle_tollerance = 1

        self.right_mid_bound = 0
        self.right_left_bound = 20
        self.right_right_bound = -20

        self.left_mid_bound = 0
        self.left_left_bound = 20
        self.left_right_bound = -20

        self.load_limits()

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
        self.lidar_mask_open_channel = 6
        self.lidar_mask_close_channel = 7
        # 运行指示灯和错误指示灯
        self.run_indicator_channel = 15
        self.fault_indicator_channel = 14

        # Scan device
        serial_numbers = (c_int * 20)()
        ret = UsbDevice_Scan(byref(serial_numbers))
        if 0 > ret:
            raise DIO_Error("DIO Init Error 1")
        elif ret == 0:
            raise DIO_Error("DIO Init Error 2")
        elif self.sn not in list(serial_numbers):
            raise DIO_Error("DIO Init Error 3")

    def reset_pid_controller(self):
        if self.mode_state_machine.state == 1 or self.mode_state_machine.state == -1:
            self.pid_controller = PIDController(self.kp, self.ki, self.kd, self.mode_state_machine.state)

    def control_left_oil_cylinder(self, direction):
        if direction == 1:
            # 左油缸收缩
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Left Oil Cylinder Error")
        elif direction == -1:
            # 左油缸拉伸
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Left Oil Cylinder Error")
        else:
            ret1 = IO_WritePin(self.sn, self.cylinder_ll_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_lr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Left Oil Cylinder Error")

    def control_right_oil_cylinder(self, direction):
        if direction == 1:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Right Oil Cylinder Error")
        elif direction == -1:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Right Oil Cylinder Error")
        else:
            ret1 = IO_WritePin(self.sn, self.cylinder_rl_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.cylinder_rr_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Right Oil Cylinder Error")

    def control_motor(self, mode):
        if mode == 1:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.active_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Motor Error")
        elif mode == -1:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.active_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Motor Error")
        elif mode == 0:
            ret1 = IO_WritePin(self.sn, self.motor_f_output_channel, self.inactive_state)
            ret2 = IO_WritePin(self.sn, self.motor_b_output_channel, self.inactive_state)
            if 0 > ret1 or 0 > ret2:
                raise DIO_Error("Control Motor Error")

    def set_motor(self):
        if not self.error_state_flag:
            control_b_input_state = c_int()
            control_f_input_state = c_int()
            control_stop_input_state = c_int()
            ret1 = IO_ReadPin(self.sn, self.control_b_input_channel, byref(control_b_input_state))
            ret2 = IO_ReadPin(self.sn, self.control_f_input_channel, byref(control_f_input_state))
            ret3 = IO_ReadPin(self.sn, self.control_stop_input_channel, byref(control_stop_input_state))
            if 0 > ret1 or 0 > ret2 or 0 > ret3:
                raise DIO_Error("Read Control Input Error")
            else:
                mode_exchange = False

                if control_f_input_state.value == self.active_state:
                    # logger.info("按前进键")
                    mode_exchange = self.mode_state_machine.transition_to_forward()
                elif control_b_input_state.value == self.active_state:
                    # logger.info("按后退键")
                    mode_exchange = self.mode_state_machine.transition_to_backward()
                elif control_stop_input_state.value == self.active_state:
                    # logger.info("按停止键")
                    mode_exchange = self.mode_state_machine.reset_to_stop()
                    
                # 如果状态转换成功
                if mode_exchange:
                    # 前进或后退模式
                    if control_f_input_state.value == self.active_state or control_b_input_state.value == self.active_state:
                        # 打开雷达罩子
                        self.open_lidar_mask()
                        # 是否开启电机有待观察
                        logger.info("电机待启动")
                        self.motor_activate = True
                        self.drive_last_sync_time = time.time()
                        # 重置pid控制器
                        self.reset_pid_controller()

                    # 停止模式
                    elif control_stop_input_state.value == self.active_state:
                        logger.info("电机停止")
                        # 重置传感器状态检查器
                        # if self.check_timer is not None:
                        #     self.check_timer.cancel()
                        # 重置停止调整计数器
                        self.stop_adjust_count = 0

                        # 清除一次过往图像
                        data = {'target': 'clear'}
                        self.thread.plotUpdateSignal.emit(data)

                        data = {'target': 'main_program', 'main_program_state': '停止模式'}
                        self.thread.plotUpdateSignal.emit(data)

                        self.sensors_health_count = 0
                        self.motor_activate = False
                        self.sensor_is_ready = False

                        self.control_motor(self.mode_state_machine.state)

                        # 关闭雷达罩子时保证左右油缸静止
                        self.control_left_oil_cylinder(0)
                        self.control_right_oil_cylinder(0)
                        self.close_lidar_mask()

                        # 更新停止状态下传感器最后更新时间
                        # self.sensor_last_update['left_angle'] = time.time()
                        # self.sensor_last_update['right_angle'] = time.time()
                        self.stop_last_sync_time = time.time()

                # 启动前检查传感器数据是否到位
                if self.motor_activate:
                    if self.sensor_is_ready:
                        logger.info("传感器数据正常，启动电机")
                        if self.mode_state_machine.state == 1:
                            data = {'target': 'main_program', 'main_program_state': '前进模式'}
                        elif self.mode_state_machine.state == -1:
                            data = {'target': 'main_program', 'main_program_state': '后退模式'}
                        self.thread.plotUpdateSignal.emit(data)

                        self.motor_activate = False
                        self.sensors_health_count = 0
                        self.control_motor(self.mode_state_machine.state)
                        
                    # else:
                    #     self.sensors_health_count += 1
                    #     if self.sensors_health_count > 40:
                    #         if not self.error_state_flag:
                    #             logger.error("启动时传感器超时")
                    #             self.error_state_handler()
                    #             self.error_state_flag = True

    def check_sensor_data_when_drive(self):
        if not self.error_state_flag and self.mode_state_machine.state != 0:
            logger.debug("check sensor data when drive, last sync time: %s", str(self.drive_last_sync_time))

            if time.time() - self.drive_last_sync_time > self.sensor_when_normal_timeout:
                logger.error("传感器运行时超时")
                current = time.time()
                for sensor, last_time in self.sensor_last_update.items():
                    diff = current - last_time
                    logger.info(f"{sensor} 上次更新: {float(diff)}秒前")

                self.error_state_handler()
                self.error_state_flag = True

    def check_sensor_data_when_stop(self):
        if not self.error_state_flag and self.mode_state_machine.state == 0:
            logger.debug("check sensor data when stop, last sync time: %s", str(self.stop_last_sync_time))

            if time.time() - self.stop_last_sync_time > self.sensor_when_normal_timeout:
                logger.error("传感器停止时超时")
                current = time.time()
                for sensor, last_time in self.sensor_last_update.items():
                    diff = current - last_time
                    logger.info(f"{sensor} 上次更新: {float(diff)}秒前")

                self.error_state_handler()
                self.error_state_flag = True

    def open_lidar_mask(self):
        logger.info("开启雷达罩子，给电3s")
        ret1 = IO_WritePin(self.sn, self.lidar_mask_open_channel, self.active_state)
        ret2 = IO_WritePin(self.sn, self.lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2:
            raise DIO_Error("Open Lidar Mask Error")
        time.sleep(1.5)
        ret1 = IO_WritePin(self.sn, self.lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2:
            raise DIO_Error("Open Lidar Mask Error")

    def close_lidar_mask(self):
        logger.info("关闭雷达罩子，给电3s")
        ret1 = IO_WritePin(self.sn, self.lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.lidar_mask_close_channel, self.active_state)
        if 0 > ret1 or 0 > ret2:
            raise DIO_Error("Close Lidar Mask Error")
        time.sleep(1.5)
        ret1 = IO_WritePin(self.sn, self.lidar_mask_open_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.lidar_mask_close_channel, self.inactive_state)
        if 0 > ret1 or 0 > ret2:
            raise DIO_Error("Close Lidar Mask Error")

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

                self.drive_last_sync_time = time.time()

                logger.debug("-----------------------")
                logger.debug("current mode:%s", self.mode_state_machine.state)

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
                    self.thread.plotUpdateSignal.emit(data)
                    # 更新拉线传感器数据
                    data = {'target': 'both_bridge', 'left_state': float(left_angle_dis),
                            'right_state': float(right_angle_dis)}
                    self.thread.plotUpdateSignal.emit(data)

                    self.draw_all_count = 0
                    
                logger.debug("front_middle_diff:%s, back_middle_diff:%s", float(front_middle_diff), -float(back_middle_diff))

                data = {'target': 'front_lidar_offset', 'offset': float(front_middle_diff)}
                self.thread.plotUpdateSignal.emit(data)

                data = {'target': 'back_lidar_offset', 'offset': -float(back_middle_diff)}
                self.thread.plotUpdateSignal.emit(data)

                if self.use_pid:
                    target_angle = self.pid_controller.pid_handle_drive_state(front_middle_diff, back_middle_diff)
                else:
                    target_angle = self.pid_controller.bang_handle_drive_state(front_middle_diff, back_middle_diff)

                logger.debug("target_angle:%s, current_left_angle:%s, current_right_angle:%s", target_angle, left_angle_dis, right_angle_dis)

                self.steer_to_target_angle_dis(target_angle, left_angle_dis, right_angle_dis)

    def steer_when_stop(self, left_angle_msg, right_angle_msg):
        if not self.error_state_flag and self.mode_state_machine.state == 0:
            # current_time = time.time()
            
            # # 更新左角度传感器的最后更新时间
            # self.sensor_last_update['left_angle'] = current_time
            # # 更新右角度传感器的最后更新时间
            # self.sensor_last_update['right_angle'] = current_time
            self.stop_last_sync_time = time.time()

            # 按停止键调整轮胎居中
            target_angle = 0
            left_angle_dis = left_angle_msg.data
            right_angle_dis = right_angle_msg.data
            
            # 更新拉线传感器数据
            data = {'target': 'both_bridge', 'left_state': float(left_angle_dis),
                    'right_state': float(right_angle_dis)}
            self.thread.plotUpdateSignal.emit(data)
            self.steer_to_target_angle_dis(target_angle, left_angle_dis, right_angle_dis)

    def check_sensor_timeout(self):
        if self.error_state_flag:  # 错误状态
            current_time = time.time()
            for sensor, last_update in self.sensor_last_update.items():
                if last_update is None or current_time - last_update > self.sensor_when_error_timeout:
                    if sensor == 'left_angle':
                        data = {'target': 'left_bridge', 'state': None}
                    elif sensor == 'right_angle':
                        data = {'target': 'right_bridge', 'state': None}
                    elif sensor == 'front_lidar':
                        data = {'target': 'front', 'f_t_points': None, 'f_t_refer_points': None,
                                'f_average_y_upper_line': None, 'f_average_y_lower_line': None,
                                'f_average_x_upper_line': None}
                        self.thread.plotUpdateSignal.emit(data)
                        data = {'target': 'front_lidar_offset', 'offset': None}
                    elif sensor == 'back_lidar':
                        data = {'target': 'back', 'b_t_points': None, 'b_t_refer_points': None,
                                'b_average_y_upper_line': None, 'b_average_y_lower_line': None,
                                'b_average_x_upper_line': None}
                        self.thread.plotUpdateSignal.emit(data)
                        data = {'target': 'back_lidar_offset', 'offset': None}
                    self.thread.plotUpdateSignal.emit(data)
                # elif self.mode_state_machine.state == 0:  # 停止模式
                #     if sensor in ['left_angle', 'right_angle']:
                #         logger.error("停止模式下传感器数据超时")
                #         if not self.error_state_flag:
                #             self.error_state_handler()
                #             self.error_state_flag = True

    def error_state_left_angle_callback(self, msg):
        current_time = time.time()
        self.sensor_last_update['left_angle'] = current_time
        if self.error_state_flag:
            self.draw_left_angle_count += 1
            if self.draw_left_angle_count == 10:
                left_angle_dis = msg.data
                data = {'target': 'left_bridge', 'state': float(left_angle_dis)}
                self.thread.plotUpdateSignal.emit(data)
                self.draw_left_angle_count = 0

    def error_state_right_angle_callback(self, msg):
        current_time = time.time()
        self.sensor_last_update['right_angle'] = current_time
        if self.error_state_flag:
            self.draw_right_angle_count += 1
            if self.draw_right_angle_count == 10:
                right_angle_dis = msg.data
                data = {'target': 'right_bridge', 'state': float(right_angle_dis)}
                self.thread.plotUpdateSignal.emit(data)
                self.draw_right_angle_count = 0

    def error_state_front_lidar_callback(self, msg):
        current_time = time.time()
        self.sensor_last_update['front_lidar'] = current_time
        if self.error_state_flag:
            self.draw_front_lidar_count += 1
            if self.draw_front_lidar_count == 10:
                front_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
                if front_lidar_points.size == 0:
                    pass
                else:
                    front_middle_diff, forward_front_diff, f_t_points, f_t_refer_points, f_average_y_upper_line, \
                            f_average_y_lower_line, f_average_x_upper_line = self.utils.get_diff(front_lidar_points)
                    data = {'target': 'front', 'f_t_points': f_t_points, 'f_t_refer_points': f_t_refer_points,
                            'f_average_y_upper_line': f_average_y_upper_line, 'f_average_y_lower_line': f_average_y_lower_line,
                            'f_average_x_upper_line': f_average_x_upper_line}
                    self.thread.plotUpdateSignal.emit(data)

                    data = {'target': 'front_lidar_offset', 'offset': float(front_middle_diff)}
                    self.thread.plotUpdateSignal.emit(data)

                self.draw_front_lidar_count = 0

    def error_state_back_lidar_callback(self, msg):
        current_time = time.time()
        self.sensor_last_update['back_lidar'] = current_time
        if self.error_state_flag:
            self.draw_back_lidar_count += 1
            if self.draw_back_lidar_count == 10:
                back_lidar_points = np.array(list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)), dtype=np.float32)
                if back_lidar_points.size == 0:
                    pass
                else:
                    back_middle_diff, backward_front_diff, b_t_points, b_t_refer_points, b_average_y_upper_line, \
                            b_average_y_lower_line, b_average_x_upper_line = self.utils.get_diff(back_lidar_points)
                    data = {'target': 'back', 'b_t_points': b_t_points, 'b_t_refer_points': b_t_refer_points,
                            'b_average_y_upper_line': b_average_y_upper_line, 'b_average_y_lower_line': b_average_y_lower_line,
                            'b_average_x_upper_line': b_average_x_upper_line}
                    self.thread.plotUpdateSignal.emit(data)

                    data = {'target': 'back_lidar_offset', 'offset': -float(back_middle_diff)}
                    self.thread.plotUpdateSignal.emit(data)

                self.draw_back_lidar_count = 0

    def steer_to_target_angle_dis(self, target_angle, left_angle_dis, right_angle_dis):
        left_cylinder_target_direction = self.left_get_target_direction(target_angle, left_angle_dis)
        right_cylinder_target_direction = self.right_get_target_direction(target_angle, right_angle_dis)
        if self.mode_state_machine.state == 0:
            if self.stop_adjust_count == 20:
                left_cylinder_target_direction = 0
                right_cylinder_target_direction = 0
            else:
                self.stop_adjust_count = self.stop_adjust_count + 1

        logger.debug('Published left oil cylinder direction: %s, Published right oil cylinder direction: %s',
                        left_cylinder_target_direction, right_cylinder_target_direction)

        self.control_left_oil_cylinder(left_cylinder_target_direction)
        self.control_right_oil_cylinder(right_cylinder_target_direction)

    def left_get_target_direction(self, target_angle, current_angle):
        if -self.angle_tollerance <= current_angle <= self.angle_tollerance:
            current_angle = 0
        elif current_angle < 0:
            current_angle = -current_angle/self.left_right_bound*50
        else:
            current_angle = current_angle/self.left_left_bound*50

        if current_angle == target_angle:
            return int(0)
        elif current_angle < target_angle:
            return int(1)
        else:
            return int(-1)

    def right_get_target_direction(self, target_angle, current_angle):
        if -self.angle_tollerance <= current_angle <= self.angle_tollerance:
            current_angle = 0
        elif current_angle < 0:
            current_angle = -current_angle/self.right_right_bound*50
        else:
            current_angle = current_angle/self.right_left_bound*50

        if current_angle == target_angle:
            return int(0)
        elif current_angle < target_angle:
            return int(1)
        else:
            return int(-1)

    def error_state_handler(self):
        logger.info("主程序停止")
        # 清除一次过往图像
        data = {'target': 'clear'}
        self.thread.plotUpdateSignal.emit(data)
        # 停止计时器
        if self.timer is not None:
            self.timer.cancel()

        if self.check_sensor_when_stop_timer is not None:
            self.check_sensor_when_stop_timer.cancel()

        if self.check_sensor_when_drive_timer is not None:
            self.check_sensor_when_drive_timer.cancel()

        # if self.check_timer is not None:
        #     self.check_timer.cancel()
        # 停止一切驱动输出
        for i in range(10):
            ret = IO_WritePin(self.sn, i, self.inactive_state)
            if ret<0:
                raise DIO_Error("停止驱动失败")

        # 关闭运行指示灯并打开错误指示灯
        ret1 = IO_WritePin(self.sn, self.run_indicator_channel, self.inactive_state)
        ret2 = IO_WritePin(self.sn, self.fault_indicator_channel, self.active_state)
        if 0 > ret1 or 0 > ret2:
            raise DIO_Error("错误指示灯初始化失败")

        data = {'target': 'main_program', 'main_program_state': '运行故障'}
        self.thread.plotUpdateSignal.emit(data)

        self.thread.display_error.emit("sensor")

    def updateSettings(self, data):
        if data['target'] == "left_settings":
            if data["left_bridge_status"] <= 0 or data["right_bridge_status"] <= 0:
                self.thread.display_error.emit("left_settings")
            else:
                self.left_left_bound = data["left_bridge_status"]
                self.right_left_bound = data["right_bridge_status"]
        elif data['target'] == "center_settings":
            msg = Empty()
            self.reset_publisher.publish(msg)
        elif data['target'] == "right_settings":
            if data["left_bridge_status"] >= 0 or data["right_bridge_status"] >= 0:
                self.thread.display_error.emit("right_settings")
            else:
                self.left_right_bound = data["left_bridge_status"]
                self.right_right_bound = data["right_bridge_status"]

        self.save_limits()

        data = {'target':'left_bridge_settings', 'settings':(self.left_left_bound, self.left_mid_bound, self.left_right_bound)}
        self.thread.plotUpdateSignal.emit(data)

        data = {'target':'right_bridge_settings', 'settings':(self.right_left_bound, self.right_mid_bound, self.right_right_bound)}
        self.thread.plotUpdateSignal.emit(data)

    def load_limits(self):
        if os.path.exists(self.config_file) and os.path.getsize(self.config_file) > 0:
            with open(self.config_file, 'r') as f:
                try:
                    limits = json.load(f)
                    self.left_left_bound = limits['left_left_bound']
                    self.right_left_bound = limits['right_left_bound']
                    self.left_mid_bound = limits['left_mid_bound']
                    self.right_mid_bound = limits['right_mid_bound']
                    self.left_right_bound = limits['left_right_bound']
                    self.right_right_bound = limits['right_right_bound']

                    data = {'target':'left_bridge_settings', 'settings':(self.left_left_bound, self.left_mid_bound, self.left_right_bound)}
                    self.thread.plotUpdateSignal.emit(data)

                    data = {'target':'right_bridge_settings', 'settings':(self.right_left_bound, self.right_mid_bound, self.right_right_bound)}
                    self.thread.plotUpdateSignal.emit(data)
                except json.JSONDecodeError:
                    logger.error("配置文件格式不正确")
        else:
            logger.info("配置文件不存在或为空")

    def save_limits(self):
        limits = {
            'left_left_bound': self.left_left_bound,
            'right_left_bound': self.right_left_bound,
            'left_mid_bound': self.left_mid_bound,
            'right_mid_bound': self.right_mid_bound,
            'left_right_bound': self.left_right_bound,
            'right_right_bound': self.right_right_bound
        }
        with open(self.config_file, 'w') as f:
            json.dump(limits, f)
            f.flush()
            os.fsync(f.fileno())
            

def main():
    app = QApplication(sys.argv)

    lidar_data_handler_thread = LidarDataHandlerThread()
    window = MainWindow()

    # 连接初始化错误信号
    lidar_data_handler_thread.initializationError.connect(lidar_data_handler_thread.handle_initialization_error)

    # 连接其他信号
    lidar_data_handler_thread.plotUpdateSignal.connect(window.update_plot_data)
    lidar_data_handler_thread.display_error.connect(window.display_error_window)

    # 初始化 LidarDataHandler
    lidar_data_handler_thread.initialize()

    # 只有在初始化成功时才连接设置信号
    if lidar_data_handler_thread.lidar_data_handler is not None:
        window.settingsSignal.connect(lidar_data_handler_thread.lidar_data_handler.updateSettings)
        lidar_data_handler_thread.start()

    # 信号处理函数,参数是必须的！
    def signal_handler(signum, frame):
        lidar_data_handler_thread.stop()
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)

    window.showFullScreen()
    exit_code = app.exec_()
    logger.info("退出qt事件循环")
    lidar_data_handler_thread.stop()
    logger.info("终止ros2线程运行")
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
