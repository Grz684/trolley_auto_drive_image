from ctypes import *
import rclpy
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from rclpy.node import Node
from linear_sensor_msgs.msg import LinearSensorData
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_sensor_data
import logging
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LeftSensor(Node):
    def __init__(self):
        super().__init__('left_sensor')
        self.timer_period_sec = 0.1
        self.current_left_angle = self.create_publisher(LinearSensorData, 'left_angle', qos_profile=qos_profile_sensor_data)
        self.left_port = "/dev/usb_port_3"
        self.debug = False
        self.connection_retry_interval = 1  # 重连间隔（秒）
        self.max_consecutive_errors = 5  # 连续错误阈值
        self.consecutive_errors = 0  # 连续错误计数
        
        self.setup_modbus_client()

        # 创建定时器，每0.1秒发布一次数据
        self.timer = self.create_timer(self.timer_period_sec, self.process_and_publish_distance)

        self.reset_subscription = self.create_subscription(
            Empty,
            'reset_encoder',
            self.reset_encoder_callback,
            10)

    def setup_modbus_client(self):
        """初始化和连接Modbus客户端"""
        while True:
            try:
                self.left_client = ModbusClient(
                    method='rtu',
                    port=self.left_port,
                    baudrate=9600,
                    stopbits=1,
                    bytesize=8,
                    parity='N',
                    timeout=1
                )
                if self.left_client.connect():
                    logger.info("连接到左车桥角度传感器成功")
                    self.consecutive_errors = 0  # 重置错误计数
                    break
                else:
                    logger.error("无法连接到左车桥角度传感器，将在 %d 秒后重试", self.connection_retry_interval)
                    time.sleep(self.connection_retry_interval)
            except Exception as e:
                logger.error("连接过程中发生错误: %s，将在 %d 秒后重试", str(e), self.connection_retry_interval)
                time.sleep(self.connection_retry_interval)

    def reconnect(self):
        """重新连接Modbus客户端"""
        try:
            self.left_client.close()
        except:
            pass
        logger.info("尝试重新连接...")
        self.setup_modbus_client()

    def get_dis(self, client):
        try:
            response = client.read_holding_registers(0x0000, 1, unit=1)
            if response.isError():
                self.consecutive_errors += 1
                logger.error("读取左车桥角度传感器错误：%s (连续错误次数: %d)", response, self.consecutive_errors)
                if self.consecutive_errors >= self.max_consecutive_errors:
                    logger.error("达到最大连续错误次数，尝试重新连接")
                    self.reconnect()
                return None
            
            self.consecutive_errors = 0  # 成功读取，重置错误计数
            raw_value = response.registers[0]
            resolution = 1024
            angle = raw_value * 360 / resolution
            
            if angle > 180:
                angle = angle - 360
            
            return angle

        except Exception as e:
            self.consecutive_errors += 1
            logger.error("读取传感器时发生错误: %s (连续错误次数: %d)", str(e), self.consecutive_errors)
            if self.consecutive_errors >= self.max_consecutive_errors:
                logger.error("达到最大连续错误次数，尝试重新连接")
                self.reconnect()
            return None

    def process_and_publish_distance(self):
        try:
            left_angle_dis = self.get_dis(self.left_client)

            if left_angle_dis is not None:
                left_angle = LinearSensorData()
                left_angle.header.stamp = self.get_clock().now().to_msg()
                left_angle.data = float(left_angle_dis)

                logger.debug("当前左车桥角度传感器数据: %.2f度", left_angle.data)
                self.current_left_angle.publish(left_angle)
        except Exception as e:
            logger.error("处理和发布数据时发生错误: %s", str(e))

    def reset_encoder_callback(self, msg):
        try:
            logger.info("收到置零命令，正在设置编码器零点")
            result = self.left_client.write_register(address=8, value=1, unit=1)
            
            if not result.isError():
                logger.info("零点设置命令已完成")
            else:
                logger.error("设置零点失败: %s", result)
        except Exception as e:
            logger.error("设置零点时发生错误: %s", str(e))

    def destroy_node(self):
        try:
            self.left_client.close()
        except:
            pass
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    left_sensor = LeftSensor()

    while True:
        try:
            rclpy.spin(left_sensor)
        except KeyboardInterrupt:
            logger.info("左车桥角度传感器节点已终止")
            break
        except Exception as e:
            logger.error("左车桥角度传感器节点错误: %s", str(e))
            logger.info("1秒后尝试恢复运行...")
            time.sleep(1)
            continue
        
    left_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()