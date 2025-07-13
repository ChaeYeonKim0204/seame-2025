import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

x_center = 80  # 영상 중앙 x좌표 기준값 (160픽셀 영상 기준)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.callback,
            qos_profile_sensor_data 
        )
        self.steering_pub = self.create_publisher(Float32, '/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.cb = CvBridge()

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        
        binary = detect_stop_line(original_img)
        img_mask = region_of_interest(binary)
        lines = houghLines(img_mask)

        if lines is not None and len(lines) > 0:
            right_lines, left_lines = separateLine(lines)
            steering, throttle = compute_control(original_img, left_lines, right_lines)
            self.get_logger().info(f"steering: {steering:.2f}, throttle: {throttle:.2f}")
            
            self.publish_controls(steering, throttle)
        else:
            self.get_logger().info("차선 없음: 정지")
            self.publish_controls(0.0, 0.0)

        cv2.imshow("original", original_img)
        cv2.imshow("binary", binary)
        cv2.imshow("roi", img_mask)
        cv2.waitKey(1)

    def publish_controls(self, steering, throttle):
        # 이미 -1.0 ~ 1.0 범위면 변환 필요 없음
        normalized_steering = float(np.clip(steering, -1.0, 1.0))
        normalized_throttle = float(np.clip(throttle, 0.0, 0.5))

        msg_s = Float32()
        msg_s.data = normalized_steering
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = normalized_throttle
        self.throttle_pub.publish(msg_t)

def main():
    rp.init()
    image_subscriber = ImageSubscriber()
    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown()

class JdOpencvLaneDetect(object):
    def __init__(self):
        self.curr_steering_angle = 90

def detect_stop_line(original_img):
    gray = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    return binary

def region_of_interest(binary):
    height, width = binary.shape
    mask = np.zeros_like(binary)
    polygon = np.array([[
        (0, height * 0.5),
        (width, height * 0.5),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(binary, mask)

def houghLines(img_mask):
    lines = cv2.HoughLinesP(img_mask, 1, np.pi/180, threshold=20, minLineLength=10, maxLineGap=20)
    if lines is not None:
        return [line[0].tolist() for line in lines]

def separateLine(lines):
    right_lines, left_lines = [], []
    slope_thresh = 0.3

    for x1, y1, x2, y2 in lines:
        if (x2 - x1) == 0:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < slope_thresh:
            continue

        if slope > 0 and x1 < x_center and x2 < x_center:
            left_lines.append([x1, y1, x2, y2])
        elif slope < 0 and x1 > x_center and x2 > x_center:
            right_lines.append([x1, y1, x2, y2])
    
    return right_lines, left_lines

def compute_control(img, left_lines, right_lines):
    """
    두 차선의 중심 기준으로 steering (-1.0 ~ 1.0), throttle (0.0 ~ 0.5) 비율 반환
    """
    height, width, _ = img.shape
    center_x = width // 2

    if not left_lines and not right_lines:
        return 0.0, 0.0  # 정지

    def average_line(lines):
        x_coords, y_coords = [], []
        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]
        if not x_coords or not y_coords:
            return None
        poly = np.polyfit(y_coords, x_coords, 1)
        y1, y2 = height, int(height * 0.6)
        x1 = int(np.polyval(poly, y1))
        x2 = int(np.polyval(poly, y2))
        return [x1, y1, x2, y2]

    left_fit = average_line(left_lines)
    right_fit = average_line(right_lines)

    if left_fit and right_fit:
        mid_x = (left_fit[0] + right_fit[0]) // 2
    elif left_fit:
        mid_x = left_fit[0] + 100
    elif right_fit:
        mid_x = right_fit[0] - 100
    else:
        mid_x = center_x

    error = mid_x - center_x

    # steering: -1.0 (좌) ~ 1.0 (우)
    steering = np.clip(error / center_x, -1.0, 1.0)

    # throttle: error 작으면 빠르게, 크면 천천히 → 최대 0.5
    throttle = 0.5 if abs(error) < 20 else 0.3
    throttle = np.clip(throttle, 0.0, 0.5)

    return steering, throttle

if __name__ == "__main__":
	main()