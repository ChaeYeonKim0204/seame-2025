# lane_detection_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 여기에 네 코드로 처리한 결과를 추가
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 70])
        upper_white = np.array([131, 255, 255])
        mask = cv2.inRange(hsv_img, lower_white, upper_white)
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        lines = detect_line_segments(blur)
        lane_lines = average_slope_intercept(frame, lines)
        line_image = display_lines(frame, lane_lines)

        combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        cv2.imshow("Lane Detection", combo_image)
        cv2.waitKey(1)  # 1ms delay

# 아래는 너가 위에 작성한 함수들 넣으면 됨 (detect_line_segments, average_slope_intercept 등)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()