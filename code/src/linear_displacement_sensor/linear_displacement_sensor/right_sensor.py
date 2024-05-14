from ctypes import *
import os
import signal
import sys
import time
import rclpy
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from rclpy import Future
from rclpy.node import Node
from linear_sensor_msgs.msg import LinearSensorData
from time import sleep
from rclpy.qos import qos_profile_sensor_data


class RightSensor(Node):
    def __init__(self):
        super().__init__('right_sensor')
        self.timer_period_sec = 0.1
        self.current_right_angle = self.create_publisher(LinearSensorData, 'right_angle', qos_profile=qos_profile_sensor_data)
        self.zero_position = 0
        self.right_port = "/dev/usb_port_10"
        self.debug = False
        # 角度传感器预设值，需校准确定
        self.mid_lower_bound = 75
        self.mid_upper_bound = 85

        # 配置MODBUS串行客户端，使用COM4端口
        self.right_client = ModbusClient(method='rtu', port=self.right_port, baudrate=9600, stopbits=1, bytesize=8,
                                         parity='N', timeout=1)
        right_conn = self.right_client.connect()

        if right_conn:
            print("Connected to the sensor")
        else:
            print("Failed to connect to the sensor")
            exit()

        # 角度传感器预设值，需校准确定
        self.mid_dis = 53
        self.max_angle = 20
        self.max_range = 125

        # 创建定时器，每0.5秒发布一次数据
        self.timer = self.create_timer(self.timer_period_sec, self.process_and_publish_distance)

    def get_dis(self, client):
        # 读取编码器值（从寄存器地址0x0000开始读取两个寄存器）
        response = client.read_holding_registers(0x0000, 2, unit=1)
        if response.isError():
            print("Error reading the encoder value:", response)
            return -100
        else:
            # 将两个16位寄存器的值合并为一个32位的值
            high_register = response.registers[0]
            low_register = response.registers[1]
            # print(f"high: {high_register}")
            # print(f"low: {low_register}")
            X = (high_register << 16) + low_register
            L = (X - self.zero_position) * 100 / 1024
            return L

    def process_and_publish_distance(self):
        right_angle = LinearSensorData()
        right_angle.header.stamp = self.get_clock().now().to_msg()

        # 左转到底122mm，右转到底22mm，中间值为72（已舍弃该参数，仅作参考）

        right_angle_dis = self.get_dis(self.right_client)

        # if right_angle_dis != -100:
        #     if self.mid_lower_bound <= right_angle_dis <= self.mid_upper_bound:
        #         right_angle.data = 0
        #     elif right_angle_dis < self.mid_lower_bound:
        #         # 当前偏右
        #         right_angle.data = -1
        #     else:
        #         # 当前偏左
        #         right_angle.data = 1

        right_angle.data = float(right_angle_dis)

        if self.debug:
            print(f"current_right_angle_dis: {right_angle_dis}mm")

        self.current_right_angle.publish(right_angle)

    def destroy_node(self):
        try:
            self.right_client.close()
            # self.right_client.close()
        finally:
            # 确保调用父类的 destroy_node
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    right_sensor = RightSensor()

    try:
        rclpy.spin(right_sensor)
    except KeyboardInterrupt:
        print("右拉线传感器关闭")
    except Exception as e:
        print(f"Caught exception: {e}")
    finally:
        right_sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
