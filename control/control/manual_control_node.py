import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from piracer.gamepads import ShanWanGamepad
from piracer.vehicles import PiRacerPro  # 추가 필요
import pygame

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        
        # 퍼블리셔 정의
        self.publisher_throttle = self.create_publisher(Float32, '/manual_throttle', 10)
        self.publisher_steering = self.create_publisher(Float32, '/manual_steering', 10)
        # self.publisher_mode = self.create_publisher(String, 'mode', 10)  # ✅ 모드 유지용 퍼블리셔 추가

        # 게임패드, 차량 초기화
        self.gamepad = ShanWanGamepad()
        # self.car = PiRacerPro()
        # self.mode = "auto"  # 초기 모드는 auto

        # pygame 초기화
        pygame.init()
        pygame.display.set_mode((1, 1))  # pygame 오류 방지용 더미 창

        # 서브스크립션 및 타이머
        self.mode = self.create_subscription(String, 'mode', self.mode_callback, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

    def mode_callback(self, msg):
        self.mode = msg.data

    def timer_callback(self):
        pygame.event.pump()

        if self.mode != "manual":
            return

        # 게임패드 입력 읽기
        gamepad_input = self.gamepad.read_data()
        throttle = gamepad_input.analog_stick_right.y * 0.5
        throttle = max(min(throttle, 0.4), -0.4)  # 제한: -0.4 ~ +0.4
        steering = gamepad_input.analog_stick_left.x

        # # 수동 모드 유지 신호 발행 ✅
        # msg_mode = String()
        # msg_mode.data = "manual"
        # self.publisher_mode.publish(msg_mode)

        # 조종 명령 발행
        msg_throttle = Float32()
        msg_throttle.data = throttle
        self.publisher_throttle.publish(msg_throttle)

        msg_steering = Float32()
        msg_steering.data = steering
        self.publisher_steering.publish(msg_steering)

        self.get_logger().info(f"수동발행  throttle: {throttle:.2f}, steering: {steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    # node.car.set_throttle_percent(0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # node.car.set_throttle_percent(0)
        # node.car.set_steering_percent(0)
        print("프로그램 종료")

if __name__ == '__main__':
    main()