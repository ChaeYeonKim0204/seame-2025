#한번 고친거 고친거 7:06 - 로그 분석 기반 수정

import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/image', self.callback, qos_profile_sensor_data)
        self.steering_pub = self.create_publisher(Float32, '/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.cb = CvBridge()
        
        # 스티어링 스무딩을 위한 변수
        self.prev_steering = 0.0
        self.smoothing_factor = 0.7

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        binary = filter_colors(original_img)
        img_mask = region_of_interest(binary)
        result = selective_erosion(img_mask)
        
        # 무게중심 계산 (항상 수행)
        centroid_x, centroid_confidence = compute_lane_centroid(result, original_img)
        
        # 직선 검출
        canny_img = apply_canny(result)
        lines = houghLines(canny_img) 

        if lines:
            distributed_lines = separateLine(lines, original_img)
            represent_points, detect_code, slope = regression(distributed_lines, original_img)
            vp = compute_intersection(represent_points)
            
            # 하이브리드 방법 사용
            direction, x_offset = predicDir_improved(detect_code, slope, vp, represent_points, 
                                                   centroid_x, centroid_confidence, original_img)
            steering, throttle = compute_control_fixed(direction, x_offset, original_img)
            
            # 스무딩 적용
            steering = self.prev_steering * self.smoothing_factor + steering * (1 - self.smoothing_factor)
            self.prev_steering = steering
            
            self.publish_controls(steering, throttle)
        else:
            # 직선 검출 실패시 무게중심만 사용
            if centroid_x is not None and centroid_confidence > 0.1:
                direction, x_offset = centroid_only_control(centroid_x, original_img)
                steering, throttle = compute_control_fixed(direction, x_offset, original_img)
                
                # 스무딩 적용
                steering = self.prev_steering * self.smoothing_factor + steering * (1 - self.smoothing_factor)
                self.prev_steering = steering
                
                self.publish_controls(steering, throttle)
            else:
                self.publish_controls(0.0, 0.0)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -1.0, 1.0))
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = float(np.clip(throttle, 0.0, 0.5))
        self.throttle_pub.publish(msg_t)

def compute_lane_centroid(binary_img, original_img):
    """
    이진화된 이미지에서 차선의 무게중심을 계산
    """
    height, width = binary_img.shape[:2]
    
    # 노이즈 제거 (erosion + dilation)
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(binary_img, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # 윤곽선 검출
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if len(contours) > 0:
        # 가장 큰 윤곽선을 차선으로 간주
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 너무 작은 윤곽선은 무시
        if cv2.contourArea(largest_contour) < 100:
            return None, 0
        
        # 모멘트 계산
        M = cv2.moments(largest_contour)
        
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # 신뢰도 계산
            contour_area = cv2.contourArea(largest_contour)
            max_area = height * width * 0.3
            confidence = min(contour_area / max_area, 1.0)
            
            # 디버깅용 시각화
            cv2.circle(original_img, (cx, cy), 5, (0, 255, 255), -1)
            cv2.line(original_img, (cx, 0), (cx, height), (0, 255, 255), 2)
            cv2.drawContours(original_img, contours, -1, (0, 255, 0), 1)
            
            print(f"[CENTROID] cx: {cx}, confidence: {confidence:.2f}")
            return cx, confidence
    
    return None, 0

def predicDir_improved(detect_code, slope, vp, represent_points, centroid_x, centroid_confidence, original_img):
    """
    개선된 방향 판단 함수 (무게중심 우선 사용 + 안전성 검사 강화)
    """
    proc_height, proc_width = original_img.shape[:2]
    mid_x = proc_width // 2
    
    print(f"[DEBUG] proc_width: {proc_width}, mid_x: {mid_x}")
    
    # 무게중심이 신뢰할만하면 우선 사용
    if centroid_x is not None and centroid_confidence > 0.2:
        x_offset = centroid_x - mid_x
        print(f"[IMPROVED] 무게중심 우선 사용: centroid_x={centroid_x}, x_offset={x_offset}")
        
        # 양쪽 차선이 모두 있고 represent_points가 유효하면 검증용으로만 사용
        if (detect_code[0] == 1 and detect_code[1] == 1 and 
            represent_points is not None and
            represent_points[0] is not None and represent_points[2] is not None):
            
            center_line_x = int((represent_points[0][0] + represent_points[2][0]) / 2)
            
            # 비정상적인 값 필터링
            if abs(center_line_x) < proc_width * 3 and abs(center_line_x - mid_x) < proc_width * 2:
                line_offset = center_line_x - mid_x
                print(f"[VERIFICATION] 직선 기반: center_line_x={center_line_x}, line_offset={line_offset}")
                
                # 차이가 너무 크면 경고
                if abs(x_offset - line_offset) > proc_width * 0.15:
                    print(f"[WARNING] 무게중심과 직선 기반 차이 큼: {abs(x_offset - line_offset)}")
                    # 가중평균 사용
                    weight = min(centroid_confidence, 0.7)
                    x_offset = int(x_offset * weight + line_offset * (1-weight))
                    print(f"[BLENDED] 가중평균 적용: {x_offset}")
            else:
                print(f"[WARNING] 비정상적인 center_line_x 값 무시: {center_line_x}")
        
        # 방향 판단
        return determine_direction_simple(x_offset, proc_width), x_offset
    
    # 무게중심을 신뢰할 수 없으면 기존 방법 사용
    else:
        return predicDir_original(detect_code, slope, vp, represent_points, original_img)

def determine_direction_simple(x_offset, proc_width):
    """
    단순한 방향 판단 (오프셋 기반)
    """
    threshold = proc_width * 0.08  # 8% 정도를 임계값으로 사용
    
    if abs(x_offset) < threshold:
        return "straight"
    elif x_offset > threshold:
        return "left turn"  # 오른쪽으로 치우쳤으니 왼쪽으로 회전
    else:
        return "right turn"  # 왼쪽으로 치우쳤으니 오른쪽으로 회전

def centroid_only_control(centroid_x, original_img):
    """
    무게중심만을 이용한 제어
    """
    proc_height, proc_width = original_img.shape[:2]
    mid_x = proc_width // 2
    x_offset = centroid_x - mid_x
    
    print(f"[CENTROID_ONLY] centroid_x: {centroid_x}, mid_x: {mid_x}, x_offset: {x_offset}")
    
    direction = determine_direction_simple(x_offset, proc_width)
    return direction, x_offset

def predicDir_original(detect_code, slope, vp, represent_points, original_img):
    """
    기존 방법 (백업용) - 안전성 검사 추가
    """
    proc_height, proc_width = original_img.shape[:2]
    
    # represent_points 안전성 검사
    if represent_points is None or all(p is None for p in represent_points):
        print("대표 점 전부 None → undefined")
        return "undefined", 0

    mid_x = proc_width // 2
    expected_lane_width = proc_width * 0.25  # 25%로 줄임

    # 양쪽 차선 다 있는 경우
    if (detect_code[0] == 1 and detect_code[1] == 1 and 
        represent_points[0] is not None and represent_points[2] is not None):
        
        center_line_x = int((represent_points[0][0] + represent_points[2][0]) / 2)
        
        # 비정상적인 값 필터링
        if abs(center_line_x) > proc_width * 3 or abs(center_line_x - mid_x) > proc_width * 2:
            print(f"[WARNING] 비정상적인 center_line_x 값: {center_line_x}, 중앙값 사용")
            x_offset = 0  # 중앙값 사용
        else:
            x_offset = center_line_x - mid_x

        print(f"[ORIGINAL] dual lane detected, center_line_x={center_line_x}, x_offset={x_offset}")
        return determine_direction_simple(x_offset, proc_width), x_offset

    # 오른쪽 차선만 감지
    elif detect_code[0] == 1 and represent_points[0] is not None:
        target_x = represent_points[0][0] - expected_lane_width
        x_offset = target_x - mid_x
        
        print(f"[ORIGINAL] only RIGHT lane detected, target_x={target_x}, x_offset={x_offset}")
        return determine_direction_simple(x_offset, proc_width), x_offset

    # 왼쪽 차선만 감지
    elif detect_code[1] == 1 and represent_points[2] is not None:
        target_x = represent_points[2][0] + expected_lane_width
        x_offset = target_x - mid_x
        
        print(f"[ORIGINAL] only LEFT lane detected, target_x={target_x}, x_offset={x_offset}")
        return determine_direction_simple(x_offset, proc_width), x_offset

    else:
        print("[ORIGINAL] 차선 검출 실패")
        return "undefined", 0

def compute_control_fixed(direction, x_offset, original_img):
    """
    수정된 제어 신호 계산 (스티어링 게인 감소, throttle 증가)
    """
    if direction == "undefined":
        return 0.0, 0.0
   
    proc_height, proc_width = original_img.shape[:2]

    # 스티어링 계산 (게인 대폭 감소)
    max_offset = proc_width // 2
    steering_gain = 0.3  # 0.8에서 0.3으로 감소
    
    # 정규화된 스티어링 값
    normalized_offset = x_offset / max_offset
    steering = normalized_offset * steering_gain
    
    # 최대 스티어링 제한
    steering = float(np.clip(steering, -0.6, 0.6))

    # throttle 조정 (전체적으로 증가)
    if direction in ["left turn", "right turn"]:
        if abs(steering) > 0.4:
            throttle = 0.25  # 급한 커브 (0.15 → 0.25)
        elif abs(steering) > 0.2:
            throttle = 0.3   # 중간 커브 (0.18 → 0.3)
        else:
            throttle = 0.35  # 완만한 커브 (새로 추가)
    else:
        throttle = 0.4   # 직진 (0.2 → 0.4)

    print(f"[CONTROL] direction: {direction}, x_offset: {x_offset}, steering: {steering:.3f}, throttle: {throttle}")
    return steering, throttle

# 나머지 함수들은 기존과 동일
def main():
    rp.init()
    image_subscriber = ImageSubscriber()
    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown()

def filter_colors(original_img):
    hsv = cv2.cvtColor(original_img,cv2.COLOR_BGR2HSV)
    
    lower_white = np.array([0,0,200], dtype=np.uint8)
    upper_white = np.array([180,30,255], dtype = np.uint8)
    binary = cv2.inRange(hsv, lower_white, upper_white)
    
    return binary

def region_of_interest(binary):  
    height, width = binary.shape[:2]
    mask = np.zeros_like(binary)
     
    bottom_width_percent = 1
    top_width_percent = 0.5
    height_percent = 0.9

    bottom_left = (int(width * (0.5 - bottom_width_percent / 2)), height)
    bottom_right = (int(width * (0.5 + bottom_width_percent / 2)), height)
    top_left = (int(width * (0.5 - top_width_percent / 2)), int(height * (1 - height_percent)))
    top_right = (int(width * (0.5 + top_width_percent / 2)), int(height * (1 - height_percent)))

    polygon = np.array([[bottom_left, top_left, top_right, bottom_right]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    img_mask = cv2.bitwise_and(binary, mask)

    return img_mask

def selective_erosion(img_mask, thickness_thresh=10, erosion_iter=1):
    kernel = np.ones((3, 3), np.uint8)
    result = img_mask.copy()

    contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        
        if w > thickness_thresh:
            roi = img_mask[y:y+h, x:x+w]
            eroded_roi = cv2.erode(roi, kernel, iterations=erosion_iter)
            result[y:y+h, x:x+w] = eroded_roi

    return result
    
def apply_canny(result, low_thresh=100, high_thresh=200):
    canny = cv2.Canny(result, low_thresh, high_thresh)
    return canny

def houghLines(canny_img):
    lines = cv2.HoughLinesP(canny_img , rho=1, theta = np.pi/180, threshold=30, minLineLength=10, maxLineGap=5)
            
    if lines is not None:
        print("직선 검출 완료")
        return [line[0].tolist() for line in lines] 
    return []

def separateLine(lines, original_img):
    right_lines = []
    left_lines = []

    if not lines:
        return [[], []]

    x_coords = []
    weights = []
    for x1, y1, x2, y2 in lines:
        x_coords.extend([x1, x2])
        weights.extend([y1, y2])

    x_center = int(np.average(x_coords, weights=weights))
    print(f"[DEBUG] 동적 x_center (가중 평균): {x_center}")

    for x1, y1, x2, y2 in lines:
        if y2 - y1 == 0:
            continue

        if x1 < x_center and x2 < x_center:
            left_lines.append([x1, y1, x2, y2])
        elif x1 > x_center and x2 > x_center:
            right_lines.append([x1, y1, x2, y2])

    return [right_lines, left_lines]
        
def fit_line(original_img,distributed_lines, fin_y=100):
    height = original_img.shape[0]
    if len(distributed_lines) < 2:
        return None, None

    # 안전성 검사 추가
    try:
        vx, vy, x0, y0 = cv2.fitLine(np.array(distributed_lines), cv2.DIST_L2, 0, 0.01, 0.01)
        
        # 0으로 나누기 방지
        if abs(vx[0]) < 1e-6:
            return None, None
            
        slope = vy[0] / vx[0]
        base_point = (x0[0], y0[0])

        init_x = int(((height - base_point[1]) / slope) + base_point[0])
        fin_x = int(((fin_y - base_point[1]) / slope) + base_point[0])

        return slope, [(init_x, height), (fin_x, fin_y)]
        
    except Exception as e:
        print(f"[ERROR] fit_line 실패: {e}")
        return None, None

def regression(distriduted_lines, original_img):
    right_lines, left_lines = distriduted_lines
    height, width = original_img.shape[:2]
    slope = [None] *2
    detect_code = [None] *2
  
    represent_points = [None] * 4

    # 오른쪽 차선 처리
    right_pts = []
    if right_lines:
        for x1, y1, x2, y2 in right_lines:
            right_pts.append((x1, y1))
            right_pts.append((x2, y2))
        
        slp, points = fit_line(original_img, right_pts)
        slope[0] = slp
        if points:
            represent_points[0] = points[0]
            represent_points[1] = points[1]
            detect_code[0] = 1
        else:
            detect_code[0] = 0

    # 왼쪽 차선 처리
    left_pts = []
    if left_lines:
        for x1, y1, x2, y2 in left_lines:
            left_pts.append((x1, y1))
            left_pts.append((x2, y2))
        
        slp, points = fit_line(original_img, left_pts)
        slope[1] = slp
        if points:
            represent_points[2] = points[0]
            represent_points[3] = points[1]
            detect_code[1] = 1
        else:
            detect_code[1] = 0
            
    if all (p is None for p in represent_points):
        return None, detect_code, slope

    return represent_points, detect_code, slope

def compute_intersection(represent_points):
    # represent_points가 None인 경우 처리
    if represent_points is None:
        print("represent_points가 None")
        return None
        
    if represent_points[0] is None or represent_points[1] is  None:
        print("오른쪽 차선 없음")
        return None
    
    elif represent_points[2] is None or represent_points[3] is None:
        print("왼쪽 차선 없음")
        return None

    try:
        right_fit = np.polyfit((represent_points[0][0],represent_points[1][0]),(represent_points[0][1],represent_points[1][1]), 1)
        right_slope = right_fit[0]
        right_y = right_fit[1]

        left_fit = np.polyfit((represent_points[2][0],represent_points[3][0]),(represent_points[2][1],represent_points[3][1]), 1)
        left_slope = left_fit[0]
        left_y = left_fit[1]

        A = np.array([[right_slope, -1], [left_slope, -1]])
        B = np.array([-right_y, -left_y])
            
        vp = np.linalg.solve(A,B)
        return tuple(vp)
        
    except Exception as e:
        print(f"[ERROR] 교점 계산 실패: {e}")
        return None

def draw_detected_lines(original_img, represent_points, vp=None):
    img = original_img.copy()

    if represent_points[0] and represent_points[1]:
        cv2.line(img, represent_points[0], represent_points[1], (255, 0, 0), 3)

    if represent_points[2] and represent_points[3]:
        cv2.line(img, represent_points[2], represent_points[3], (0, 255, 0), 3)

    if vp is not None:
        vp_point = (int(vp[0]), int(vp[1]))
        cv2.circle(img, vp_point, 8, (0, 0, 255), -1)

    return img

if __name__ == "__main__":
    main()