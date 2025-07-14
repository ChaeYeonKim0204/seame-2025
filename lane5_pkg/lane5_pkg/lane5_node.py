import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/image', self.callback, qos_profile_sensor_data)
        self.steering_pub = self.create_publisher(Float32, '/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.cb = CvBridge()

    def callback(self, msg):
        # 좌우 차선용 ROI 지정 (y가 위쪽부터 순서대로 내려가는 식으로)
        left_rois = [
        (80, 320, 150, 100),
        (100, 260, 150, 100)
        ]

        right_rois = [
            (680, 320, 150, 100),
            (660, 260, 150, 100)
        ]
        orginal_msg = self.cb.imgmsg_to_cv2(msg,"bgr8")
        detect_lanes_and_center_path(orginal_msg, left_rois, right_rois, pixel_threshold=500)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -1.0, 1.0))
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = float(np.clip(throttle, 0.0, 0.5))
        self.throttle_pub.publish(msg_t)

def main():
    rp.init()
    image_subscriber = ImageSubscriber()
    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown

def detect_lanes_and_center_path(video_path, left_rois, right_rois, pixel_threshold=500):
    while True:
        if not ret:
            break
        height, width = original_img.shape[:2]
        center = width / 2
        left_points = []
        right_points = []

        def process_roi_group(rois, color, point_list):
            for roi in rois:
                x, y, w, h = roi
                roi_img = orginal_img[y:y+h, x:x+w]

                # 흰색 마스크
                lower_white = np.array([200, 200, 200], dtype=np.uint8)
                upper_white = np.array([255, 255, 255], dtype=np.uint8)
                mask = cv2.inRange(roi_img, lower_white, upper_white)
                white_pixel_count = cv2.countNonZero(mask)

                # ROI 시각화
                box_color = (255, 255, 255) if white_pixel_count > pixel_threshold else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)

                if white_pixel_count > pixel_threshold:
                    cx = x + w // 2
                    cy = y + h // 2
                    point_list.append((cx, cy))
                    cv2.circle(orginal_img, (cx, cy), 4, color, -1)

        # 좌우 차선 각각 처리
        process_roi_group(left_rois, (0, 255, 255), left_points)   # 노란 점
        process_roi_group(right_rois, (0, 255, 0), right_points)   # 초록 점

        # 각 차선 연결 선
        for pts, line_color in [(left_points, (0, 255, 255)), (right_points, (0, 255, 0))]:
            for i in range(len(pts) - 1):
                cv2.line(original_img, pts[i], pts[i+1], line_color, 2)

        # 중앙 경로 계산 (좌우 좌표가 같은 수일 필요는 없음 → 최소 길이 기준)
        center_points = []
        min_len = min(len(left_points), len(right_points))
        for i in range(min_len):
            lx, ly = left_points[i]
            rx, ry = right_points[i]
            mx = (lx + rx) // 2
            my = (ly + ry) // 2
            center_points.append((mx, my))
              

        if len(center_points) > 0:
            target_point = center_points[-1]
            center_x, center_y = target_point
            error_x = center_x - center

            if abs(error_x) < 20:
                direction = "straight"
            elif error_x < 0:
                direction = "left"
            else:
                direction = "right"

        # 중앙 경로 선으로 연결
        cv2.imshow("Lane and Center Path", frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
def compute_control(direction):
    if direction == "left":
        steering = -0.7
        throttle = 0.15
    elif direction == "right":
        steering = 0.4
        throttle = 0.15
    elif direction == "straight":
        steering = -0.25
        throttle = 0.15
    else:  # undefined
        steering = 0.0
        throttle = 0.0
    return steering, throttle


if __name__ == "__main__":
    main()

