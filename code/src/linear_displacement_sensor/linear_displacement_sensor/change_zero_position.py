from pymodbus.client import ModbusSerialClient as ModbusClient

# 配置MODBUS串行客户端，使用COM4端口
client = ModbusClient(method='rtu', port='/dev/usb_port_2', baudrate=9600, stopbits=1, bytesize=8, parity='N', timeout=1)

# 尝试与传感器建立连接
connection = client.connect()
if connection:
    print("Connected to the sensor")

    zero_position = 0

    # 将32位数值分解为两个16位的数
    high_register = zero_position >> 16
    low_register = zero_position & 0xFFFF

    # 写入寄存器
    address = 0x000B  # 起始地址
    registers = [high_register, low_register]  # 要写入的值
    response = client.write_registers(address, registers, slave=1)  # unit是从站地址

    # 检查响应
    if not response.isError():
        print("写入成功")
    else:
        print("写入失败：", response)

    client.close()
else:
    print("Failed to connect to the sensor")