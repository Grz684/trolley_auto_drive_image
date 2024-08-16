from ctypes import *
import rclpy
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from rclpy.node import Node
from linear_sensor_msgs.msg import LinearSensorData
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_sensor_data
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RightSensor(Node):
    def __init__(self):
        super().__init__('right_sensor')
        self.timer_period_sec = 0.1
        self.current_right_angle = self.create_publisher(LinearSensorData, 'right_angle', qos_profile=qos_profile_sensor_data)
        self.right_port = "/dev/usb_port_10"  # 修改为右侧传感器的端口
        self.debug = False
        
        # 配置MODBUS串行客户端
        self.right_client = ModbusClient(method='rtu', port=self.right_port, baudrate=9600, stopbits=1, bytesize=8,
                                         parity='N', timeout=1)
        right_conn = self.right_client.connect()

        if right_conn:
            logger.info("连接到右车桥角度传感器")
        else:
            raise Exception("无法连接到右车桥角度传感器")

        # 创建定时器，每0.1秒发布一次数据
        self.timer = self.create_timer(self.timer_period_sec, self.process_and_publish_distance)

        # 创建订阅者，订阅"reset_encoder"主题（用于置零操作）
        self.reset_subscription = self.create_subscription(
            Empty,
            'reset_encoder',
            self.reset_encoder_callback,
            10)

    def get_dis(self, client):
        # 读取编码器值（从寄存器地址0x0000开始读取一个寄存器）
        response = client.read_holding_registers(0x0000, 1, unit=1)
        if response.isError():
            logger.error("读取右车桥角度传感器错误：%s", response)
            return None
        else:
            raw_value = response.registers[0]
            # 计算单圈角度
            resolution = 1024  # 单圈分辨率,根据实际值修改
            angle = raw_value * 360 / resolution

            # 将角度限制在-180到180度之间
            if angle > 180:
                angle = angle - 360
            
            return angle

    def process_and_publish_distance(self):
        right_angle_dis = self.get_dis(self.right_client)

        if right_angle_dis is not None:
            right_angle = LinearSensorData()
            right_angle.header.stamp = self.get_clock().now().to_msg()
            right_angle.data = float(right_angle_dis)

            logger.debug("当前右车桥角度传感器数据: %.2f度", right_angle.data)

            self.current_right_angle.publish(right_angle)

    def reset_encoder_callback(self, msg):
        """
        接收到reset_encoder消息时的回调函数（用于置零操作）
        """
        logger.info("收到置零命令，正在设置编码器零点")
        try:
            # 写入值1到寄存器0x0008（8）
            result = self.right_client.write_register(address=8, value=1, unit=1)
            
            if not result.isError():
                logger.info("零点设置命令已完成")
        except Exception as e:
            logger.error("设置零点时发生错误: %s", e)

    def destroy_node(self):
        try:
            self.right_client.close()
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    right_sensor = RightSensor()

    try:
        rclpy.spin(right_sensor)
    except KeyboardInterrupt:
        logger.info("右车桥角度传感器节点已终止")
    except Exception as e:
        logger.error("右车桥角度传感器节点错误: %s", e)
    finally:
        right_sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()