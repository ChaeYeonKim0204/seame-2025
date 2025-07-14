import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from piracer.gamepads import ShanWanGamepad

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        self.publisher_throttle = self.create_publisher(Float32, '/manual_throttle', 10)
        self.publisher_steering = self.create_publisher(Float32, '/manual_steering', 10)
        self.gamepad = ShanWanGamepad()
        self.mode = "auto"  # 초기 모드

        # 모드 구독
        self.mode_sub = self.create_subscription(String, 'mode', self.mode_callback, 10)

        # 0.1초마다 콜백
        self.timer = self.create_timer(0.1, self.timer_callback)

    def mode_callback(self, msg):
        self.mode = msg.data

    def timer_callback(self):
        if self.mode != "manual":
            return

        gamepad_input = self.gamepad.read_data()
        throttle_tmp = gamepad_input.analog_stick_right.y * 0.5
        if throttle_tmp >= 0.3:
            throttle = 0.3
        else:
            throttle = throttle_tmp
    
        steering = gamepad_input.analog_stick_left.x

        msg_throttle = Float32()
        msg_throttle.data = throttle
        self.publisher_throttle.publish(msg_throttle)

        msg_steering = Float32()
        msg_steering.data = steering
        self.publisher_steering.publish(msg_steering)

        self.get_logger().info(f"ManualControl Published - Throttle: {throttle:.2f}, Steering: {steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
