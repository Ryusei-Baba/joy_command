import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

class JoyServiceNode(Node):

    def __init__(self):
        super().__init__('joy_command_node')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.client = self.create_client(Trigger, '/waypoint_manager2/next_wp')
        self.button_state = 0

    def joy_callback(self, msg):
        if msg.buttons[1] != self.button_state:
            self.button_state = msg.buttons[1]
            if self.button_state == 1: 
                self.call_service()

    def call_service(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available')
            return
        request = Trigger.Request()
        self.get_logger().info(f'send next_wp')
        future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = JoyServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()