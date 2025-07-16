import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from piracer.vehicles import PiRacerPro

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.piracer = PiRacerPro()
        self.mode = "auto"
        self.throttle = 0.0
        self.steering = -0.23

        # 모드 구독
        self.mode_sub = self.create_subscription(
            String,
            'mode',
            self.mode_callback,
            10
        )

        # 자동 조종 명령 구독 (추가)
        self.auto_throttle_sub = self.create_subscription(
            Float32,
            '/throttle',  # 자동 주행 노드가 발행하는 토픽
            self.auto_throttle_callback,
            10
        )
        self.auto_steering_sub = self.create_subscription(
            Float32,
            '/steering',  # 자동 주행 노드가 발행하는 토픽
            self.auto_steering_callback,
            10
        )

        # 수동 조종 명령 구독 (평상시엔 무시)
        self.manual_throttle_sub = self.create_subscription(
            Float32,
            '/manual_throttle',
            self.manual_throttle_callback,
            10
        )
        self.manual_steering_sub = self.create_subscription(
            Float32,
            '/manual_steering',
            self.manual_steering_callback,
            10
        )

        # 자동 조종을 위한 타이머 (0.02초 주기)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def mode_callback(self, msg: String):
        self.mode = msg.data
        self.get_logger().info(f'Mode switched to: {self.mode}')
        if self.mode == "auto":
            # 자동모드 진입 시 초기화(예시)
            self.throttle = 0.2  # 예: 자동 주행 기본 throttle 값
            self.steering = -0.23
        else:
            # 수동 모드 시 초기 throttle, steering 0으로 초기화 (옵션)
            self.throttle = 0.0
            self.steering = -0.23

    def manual_throttle_callback(self, msg: Float32):
        if self.mode == "manual":
            self.throttle = msg.data

    def manual_steering_callback(self, msg: Float32):
        if self.mode == "manual":
            self.steering = msg.data

    # 자동 주행 콜백 (추가됨)
    def auto_throttle_callback(self, msg: Float32):
        if self.mode == "auto":
            self.throttle = msg.data

    def auto_steering_callback(self, msg: Float32):
        if self.mode == "auto":
            self.steering = msg.data

    def timer_callback(self):
        if self.mode == "auto" and self.throttle == 0.0:
            self.get_logger().warn("자동주행 모드지만 throttle 값이 아직 안 들어옴")  # 추가함
        
        self.piracer.set_throttle_percent(self.throttle)
        self.piracer.set_steering_percent(self.steering)
        
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt. Shutting down.")
    finally:
        node.piracer.set_throttle_percent(0.0)
        node.piracer.set_steering_percent(0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
		main()