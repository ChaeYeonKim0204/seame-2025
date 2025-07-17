import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, qos_profile_sensor_data)
        self.steering_pub = self.create_publisher(Float32, '/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.cb = CvBridge()
        # 🟢 PD 제어용 상태 변수
        self.last_time = 0.0
        self.last_error = 0.0

        # 🟡 PD 계수 (원하면 ROS2 파라미터로도 설정 가능)
        self.kp = 0.4
        self.kd = self.kp * 0.65
        self.base_speed = 0.3  # throttle 기본값 (0~1 사이)

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        binary = filter_colors(original_img)
        img_mask = region_of_interest(binary)
        canny_img = apply_canny(img_mask)
        lines = detect_line_segments(canny_img)

        lane_lines = separateLine(lines, original_img)
        steering_angle = get_steering_angle(original_img, lane_lines)

        steer, throttle, self.last_error, self.last_time = compute_pd_control(steering_angle, self.last_error, self.last_time)
        self.publish_controls(steer, throttle)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -0.7, 0.7))
        print(f"[PUBLISH] Steering: {msg_s.data:.3f}")
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = float(np.clip(throttle, 0.0, 0.5))
        self.throttle_pub.publish(msg_t)
        
def main():
    rp.init()
    image_subscriber = ImageSubscriber()

    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown()

def filter_colors(original_img):
    hsv = cv2.cvtColor(original_img, cv2.COLOR_BGR2HSV)
    
    # 훨씬 보수적인 흰색 임계값
    lower_white = np.array([0, 0, 240], dtype=np.uint8)
    upper_white = np.array([180, 15, 255], dtype=np.uint8)
    
    binary = cv2.inRange(hsv, lower_white, upper_white)
    return binary


#사다리꼴로 관심영역 지정
def region_of_interest(binary):  
    height, width = binary.shape[:2]
    mask = np.zeros_like(binary)  #빈 마스크 생성 (검정색만 있는 이미지)
     
    bottom_width_percent = 1  # 하단 너비 비율
    top_width_percent = 0.8 # 상단 너비 비율
    height_percent = 0.9      # 높이 비율

    bottom_left = (int(width * (0.5 - bottom_width_percent / 2)), height)
    bottom_right = (int(width * (0.5 + bottom_width_percent / 2)), height)
    top_left = (int(width * (0.5 - top_width_percent / 2)), int(height * (1 - height_percent)))
    top_right = (int(width * (0.5 + top_width_percent / 2)), int(height * (1 - height_percent)))


    polygon = np.array([[bottom_left, top_left, top_right, bottom_right]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    img_mask = cv2.bitwise_and(binary, mask)

    #cv2.imshow("roi Image", img_mask) 
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    return img_mask

    # # 화면 아래 절반만 관심 영역으로 설정
    # polygon = np.array([[
    #     (0, height * 0.45),
    #     (width, height * 0.45),
    #     (width, height),
    #     (0, height),
    # ]], np.int32)

    # cv2.fillPoly(mask, polygon, 255)  # 관심 영역을 흰색(255)으로 채움
    
    # img_mask = cv2.bitwise_and(binary, mask)  # 관심영역만 추출
    
def apply_canny(result, low_thresh=100, high_thresh=200):
    """
    ROI 마스크 이미지에 Canny Edge Detection 적용
    """
    canny = cv2.Canny(result, low_thresh, high_thresh)
    # cv2.imshow("Canny Edge", canny)
    # cv2.waitKey(1)
    # cv2.destroyAllWindows()
    return canny


# 허프 변환에 새로운 함수 추가 
def detect_line_segments(canny_img):
    lines = cv2.HoughLinesP(canny_img , rho=1, theta = np.pi/180, threshold=30, minLineLength=10, maxLineGap=5)
            
    return lines 

# 오른쪽, 왼쪽 차선 분리
def separateLine(lines, original_img):    
    lane_lines = []

    if lines is None:
        print("no line segments detected")
        return lane_lines


    right_lines = []
    left_lines = []
    
    width = original_img.shape[1]
    boundary = 1/3    

    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in lines:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            # fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_lines.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_lines.append((slope, intercept))
   
        left_lines_average = np.average(left_lines, axis=0)
        if len(left_lines) > 0:
            lane_lines.append(make_points(original_img, left_lines_average))

        right_lines_average = np.average(right_lines, axis=0)
        if len(right_lines) > 0:
            lane_lines.append(make_points(original_img, right_lines_average))

    return lane_lines

def make_points(original_img, lines_average):
    height = original_img.shape[0]
    
    slope, intercept = lines_average
    
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

# def display_lines(original_img, lines, line_color=(0, 255, 0), line_width=6):
#     line_image = np.zeros_like(original_img)
    
#     if lines is not None:
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
#     line_image = cv2.addWeighted(original_img, 0.8, line_image, 1, 1)
    
#     return line_image

# def display_heading_line(original_img, steering_angle, line_color=(0, 0, 255), line_width=5 ):
#     heading_image = np.zeros_like(original_img)
#     height, width, _ = original_img.shape
    
#     steering_angle_radian = steering_angle / 180.0 * math.pi
    
#     x1 = int(width / 2)
#     y1 = height
#     x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
#     y2 = int(height / 2)
    
#     cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
#     heading_image = cv2.addWeighted(original_img, 0.8, heading_image, 1, 1)
    
#     return heading_image
        
def get_steering_angle(original_img, lane_lines):
    
    height,width,_ = original_img.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle


def compute_pd_control(steering_angle, last_error, last_time, kp=0.4, kd_ratio=0.65, base_speed=0.3):
    now = time.time()
    dt = now - last_time if last_time != 0 else 1e-3
    error = abs(steering_angle - 90)

    deviation = steering_angle - 90
    if -5 < deviation < 5:
        steering = 0.0
        error = 0.0
    else:
        steering = deviation / 180.0
        steering = max(min(steering, 0.5), -0.5)

    kd = kp * kd_ratio
    derivative = kd * (error - last_error) / dt
    proportional = kp * error
    pd_value = base_speed + derivative + proportional
    throttle = max(min(abs(pd_value), 1.0), 0.0)

    return steering, throttle, error, now


# def draw_detected_lines(original_img, represent_points, vp=None):
#     # 복사본 만들기
#     img = original_img.copy()

#     # 대표 차선 좌표가 있을 때만 그림
#     # 오른쪽 차선 (파란색)
#     if represent_points[0] is None or represent_points[1] is None or represent_points[2] is None:
#             print("오른쪽 차선 없음")
#             return None
        
#     # 왼쪽 차선 점들 (3, 4, 5)
#     elif represent_points[3] is None or represent_points[4] is None or represent_points[5] is None:
#         print("왼쪽 차선 없음")
#         return None
        
#     # 교차점 (빨간 점)
#     if vp is not None:
#         vp_point = (int(vp[0]), int(vp[1]))
#         cv2.circle(img, vp_point, 8, (0, 0, 255), -1)

#     return img
    



if __name__ == "__main__":
    main()
