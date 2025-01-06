import struct
import serial
import time
import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import Target 
from std_msgs.msg import String

# CRC-8 校验表
CRC08_Table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# 定义结构体
class AutoAim_Data_Rx:
    def __init__(self, Pos_x, Pos_y, Pos_z, Vel_x, Vel_y, Vel_z, tracking, shoot_freq, Aimed_ID):
        self.Pos_x = Pos_x
        self.Pos_y = Pos_y
        self.Pos_z = Pos_z
        self.Vel_x = Vel_x
        self.Vel_y = Vel_y
        self.Vel_z = Vel_z
        self.tracking = tracking
        self.shoot_freq = shoot_freq
        self.Aimed_ID = Aimed_ID

class Chassis_Data_Rx:
    def __init__(self, vx, vy, yaw_speed):
        self.vx = vx
        self.vy = vy
        self.yaw_speed = yaw_speed

class Rotate_Data_Rx:
    def __init__(self, rotate):
        self.rotate = rotate


# 定义帧头和命令字
HEADER = 0xAA
CMD_ID_AUTOAIM_DATA_RX = 0x81
CMD_ID_CHASSIS_DATA_RX = 0x82
CMD_ID_ROTATE_DATA_RX = 0x85
# 打包结构体为字节流
def pack_autoaim_data(data):
    return struct.pack('<ffffffBBB', data.Pos_x, data.Pos_y, data.Pos_z, data.Vel_x, data.Vel_y,  data.Vel_z, data.tracking, data.shoot_freq, data.Aimed_ID)

def pack_chassis_data(data):
    return struct.pack('<fff', data.vx, data.vy, data.yaw_speed)

def pack_rotate_data(data):
    return struct.pack('<h', data.rotate)

# 计算CRC校验位
def crc8(data):
    crc = 0xff
    for byte in data:
        crc = CRC08_Table[crc ^ byte]
    return crc

# 构建消息
def build_autoaim_message(data):
    data_bytes = pack_autoaim_data(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    # print(length)
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes + struct.pack('<B', crc)

def build_chassis_message(data):
    data_bytes = pack_chassis_data(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_CHASSIS_DATA_RX) + data_bytes + struct.pack('<B', crc)

def build_rotate_message(data):
    data_bytes = pack_rotate_data(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_ROTATE_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_ROTATE_DATA_RX) + data_bytes + struct.pack('<B', crc)


# 连接串口
ser = serial.Serial('/dev/ttyUSB0', 115200)  # 假设串口为COM1，波特率为9600

def send_auto_aim(Pos_x, Pos_y, Pos_z, Vel_x, Vel_y, Vel_z, tracking, shoot_freq, Aimed_ID):
    data = AutoAim_Data_Rx(Pos_x, Pos_y, Pos_z, Vel_x, Vel_y, Vel_z, tracking, shoot_freq, Aimed_ID)
    message = build_autoaim_message(data)
    ser.write(message)


def parse_message(message):
    # 检查帧头
    if message[0] != 'aa':
        raise ValueError("Invalid header")
    # 解析帧长度
    length = message[1]
    if len(message) != int(length,16):
        raise ValueError("Invalid message length")
    # 检查CRC校验位
    # crc = crc8(message[3:-1])
    # if crc != message[-1]:
    #     raise ValueError("CRC check failed")
    # 解析命令字
    cmd_id = message[2]
    result = []
    if cmd_id == '14':
    #     raise ValueError("Invalid command ID")
        for i in range(3, len(message)-2, 4):
            string = ''.join(message[i:i + 4])  # 每四个十六进制数组合成一个字符串
            bytes_data = bytes.fromhex(string)
            float_value = struct.unpack('<f', bytes_data)[0]
            result.append(float_value)
        result.append(struct.unpack('<B', bytes.fromhex(message[-2]))[0])
        return result

rx_buffer = []


class SPNode(Node):
    def __init__(self):
        super().__init__("subscriber_publisher_node")
        self.subscription = self.create_subscription(Target, '/tracker/target', self.listener_callback, rclpy.qos.qos_profile_sensor_data)  # CHANGE
        self.subscription  # 防止未使用变量def listener_callback(self, msg):警告
        self.publisherss = self.create_publisher(String,"refree",10)
    def listener_callback(self, msg):
        print(msg.position.x)
        send_auto_aim(msg.position.x, msg.position.y, msg.position.z, msg.v_yaw, msg.velocity.y, msg.velocity.z, msg.tracking, 1, 1)
        self.publish_message("1")
    def publish_message(self, data):
        msg = String()
        msg.data = data
        self.publisherss.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    subscriber_publisher_node = SPNode()
    rclpy.spin(subscriber_publisher_node)
    subscriber_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# def info_publisher():
#     # ROS节点初始化
#     rospy.init_node('send_receive', anonymous=True)
#     global rx_buffer
#     global parsed_data
#     # 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
#     person_info_pub = rospy.Publisher('/receive', gimbal, queue_size=10)

#     #设置循环的频率
#     rate = rospy.Rate(300) 

#     while not rospy.is_shutdown():
#         # 初始化learning_topic::Person类型的消息
#         head = ser.read()
#         if head.hex() == 'aa':
#             rx_buffer.append(head.hex())
#             length = ser.read()
#             rx_buffer.append(length.hex())
#             while len(rx_buffer) < int.from_bytes(length, byteorder='big'):  # 至少包含帧头、帧长度和CRC校验位的长度
#                 rx_buffer.append(ser.read().hex())
#             print(rx_buffer)
#             # rx_buffer = []
#             try:
#                 parsed_data = parse_message(rx_buffer)
#                 rx_buffer = []
#                 gimbal_msg = gimbal()
#                 gimbal_msg.Pitch = parsed_data[0]
#                 gimbal_msg.Roll  = parsed_data[1]
#                 gimbal_msg.Yaw  = parsed_data[2]
#                 gimbal_msg.fric_speed = parsed_data[3]
#                 # 发布消息
#                 person_info_pub.publish(gimbal_msg)
#             except:
#                 print("Error:")
#                 rx_buffer = []
#         # rospy.loginfo("Publsh person message[%s, %d, %d]",
#         #         person_msg.name, person_msg.age, person_msg.sex)

#         # 按照循环频率延时
#         rate.sleep()
# if __name__ == '__main__':
#     try:
#         info_publisher()
#     except rospy.ROSInterruptException:
#         pass