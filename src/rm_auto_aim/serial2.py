import rclpy
from rclpy.node import Node
# import sys
# sys.path.append("/rm_auto_aim")
from auto_aim_interfaces.msg import Target  # CHANGE
# import Target
# print(1)
class MinimalSubscriber(Node):
    '''
    订阅器节点类
    '''
    def __init__(self):
        # 初始化节点
        super().__init__('minimal_subscriber')

        # 初始化订阅器，话题类型Sphere，话题topic，回调函数listener_callback
        self.subscription = self.create_subscription(Target, 'tracker/target', self.listener_callback, rclpy.qos.qos_profile_sensor_data)  # CHANGE
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        '''
        订阅器回调函数
        '''
        # 打印订阅话题的消息数据
        self.get_logger().info('I heard: "%f"' % msg.position.x)  # CHANGE


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点
    minimal_subscriber = MinimalSubscriber()

    # 运行节点
    rclpy.spin(minimal_subscriber)

    # 销毁节点，退出ROS2
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
