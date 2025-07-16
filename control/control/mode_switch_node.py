import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame  # 조이스틱 입력 감지

class ModeSwitchNode(Node):
    def __init__(self):
        super().__init__('mode_switch_node')
        self.publisher_ = self.create_publisher(String, 'mode', 10)
        self.mode = "auto"

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("조이스틱이 연결되어 있지 않습니다!")
            exit(1)
        else:
            self.get_logger().info("조이스틱이 연결되었습니다!")  # 여기 추가

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.prev_button_state = False
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        button_pressed = self.joystick.get_button(0)  # 0번 버튼이 눌렸는지 확인

        if button_pressed and not self.prev_button_state:
            self.get_logger().info("A버튼이 눌렸습니다!")

            self.mode = "manual" if self.mode == "auto" else "auto"
            msg = String()
            msg.data = self.mode
            self.publisher_.publish(msg)
            self.get_logger().info(f'모드전환: {self.mode}')

        self.prev_button_state = button_pressed

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
