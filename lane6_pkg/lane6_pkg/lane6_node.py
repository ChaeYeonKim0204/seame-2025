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
        self.prev_vp = None
        self.prev_steer = 0.0  # 직진 = 0 기준으로 변경

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        binary = filter_colors(original_img)
        img_mask = region_of_interest(binary)
        result = selective_erosion(img_mask)
        canny_img = apply_canny(result)
        lines = houghLines(canny_img)
        if lines:
            distributed_lines = separateLine(lines, original_img)
            result = regression(distributed_lines, original_img)

            if result is not None:
                if result[0] is None:
                    self.publish_controls(-0.23, 0.0)
                    return
            else:
                return

            represent_points, detect_code, slope = result

            vp = compute_intersection(represent_points)

            if vp is not None:
                self.prev_vp = vp  # 새 교점이면 업데이트
            elif self.prev_vp is not None:
                vp = self.prev_vp  # 없으면 이전꺼 사용
            else:
                self.publish_controls(-0.23, 0.0)
                return

            vp_vehicle = convert_vp_to_vehicle_coords(vp, original_img.shape)
            target_heading = compute_target_heading(vp_vehicle)
            current_heading = compute_current_heading(self.prev_steer)
            delta_heading = compute_steering_angle(target_heading, current_heading)
            steering = steering_angle_to_steer(delta_heading)
            self.prev_steer = steering  # 내부는 0 기준 유지
            real_steer = steering - 0.23  # 실제 퍼블리시할 값

            print(f"[DEBUG] vp: {vp}, vehicle_coords: {vp_vehicle}")
            print(f"[DEBUG] target_heading: {math.degrees(target_heading):.2f} deg")
            print(f"[DEBUG] delta_heading: {math.degrees(delta_heading):.2f} deg")
            print(f"[DEBUG] steering (real): {real_steer:.3f}")

            throttle = 0.25 if abs(real_steer) < 0.4 else 0.25
            self.publish_controls(real_steer, throttle)
        else:
            self.publish_controls(-0.23, 0.0)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -1.23, 1.0))
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
    hsv = cv2.cvtColor(original_img,cv2.COLOR_BGR2HSV)
    
    #흰색 임계값(값 수정 필요)
    lower_white = np.array([0,0,200], dtype=np.uint8)
    upper_white = np.array([180,30,255], dtype = np.uint8)
    binary = cv2.inRange(hsv, lower_white, upper_white)
    
    # cv2.imshow("Binary Image", binary) # "Binary Image"는 창의 제목입니다.
    # cv2.waitKey(0)# 키보드 입력 대기. 0은 아무 키나 누를 때까지 무한 대기.
    # cv2.destroyAllWindows() # 모든 OpenCV 창 닫기
    
    return binary

#사다리꼴로 관심영역 지정
def region_of_interest(binary):  
    height, width = binary.shape[:2]
    mask = np.zeros_like(binary)  #빈 마스크 생성 (검정색만 있는 이미지)
     
    bottom_width_percent = 1  # 하단 너비 비율
    top_width_percent = 0.6 # 상단 너비 비율
    height_percent = 0.65      # 높이 비율

    bottom_left = (int(width * (0.5 - bottom_width_percent / 2)), height)
    bottom_right = (int(width * (0.5 + bottom_width_percent / 2)), height)
    top_left = (int(width * (0.5 - top_width_percent / 2)), int(height * (1 - height_percent)))
    top_right = (int(width * (0.5 + top_width_percent / 2)), int(height * (1 - height_percent)))


    polygon = np.array([[bottom_left, top_left, top_right, bottom_right]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    img_mask = cv2.bitwise_and(binary, mask)

    # cv2.imshow("roi Image", img_mask) 
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

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

def selective_erosion(img_mask, thickness_thresh=10, erosion_iter=1):
    """
    두께(또는 면적)가 일정 기준을 넘는 차선 영역에만 erosion 적용
    """
    kernel = np.ones((3, 3), np.uint8)
    result = img_mask.copy()

    contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        
        if w > thickness_thresh:  # → '두꺼운' 차선으로 판단
            # 해당 영역만 잘라서 erosion 적용
            roi = img_mask[y:y+h, x:x+w]
            eroded_roi = cv2.erode(roi, kernel, iterations=erosion_iter)
            result[y:y+h, x:x+w] = eroded_roi  # 다시 넣어줌

    # # 확인용 출력
    # cv2.imshow("Selective Erosion", result)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return result

    
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
def houghLines(canny_img):
    lines = cv2.HoughLinesP(canny_img , rho=1, theta = np.pi/180, threshold=30, minLineLength=15, maxLineGap=10)
            
    # 직선이 감지되면
    if lines is not None:
        # print("직선 검출 완료")
        # x1, y1, x2, y2을 반환
        return [line[0].tolist() for line in lines] 
    return []

# 오른쪽, 왼쪽 차선 분리
def separateLine(lines, original_img):
    right_lines = []
    left_lines = []

    width = original_img.shape[1]
    x_center = width / 2

    if not lines:
        return [[], []]

    # 가중 평균 중심 계산
    x_coords = []
    weights = []
    for x1, y1, x2, y2 in lines:
        x_coords.extend([x1, x2])
        weights.extend([y1, y2])  # 아래쪽 점일수록 더 중요한 선으로 간주

    x_center = int(np.average(x_coords, weights=weights))
    # print(f"[DEBUG] 동적 x_center (가중 평균): {x_center}")

    for i in lines:
        x1, y1, x2, y2 = i

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0:
            # 수직선: slope는 무한대 → 차선으로 쓰기 애매 → 무시
            # print("수직선 (x1 == x2), 무시")
            continue

        if dy == 0:
            # 수평선 → 정지선일 가능성 높음
            # print("수평선 (y1 == y2), 정지선 후보")
            continue

        slope = dy / dx

        # 중심선을 기준으로 양쪽에 위치한 선을 분리
        if x1 < x_center and x2 < x_center:
            left_lines.append([x1, y1, x2, y2])
        elif x1 > x_center and x2 > x_center:
            right_lines.append([x1, y1, x2, y2])

    return [right_lines, left_lines]


def safe_slope(vx, vy, epsilon=1e-6):
    if abs(vx[0]) < epsilon:
        # 수직선 또는 수직에 가까운 선
        return float('inf')
    else:
        return vy[0] / vx[0]
        
#기울기와 직선의 시작점과 끝점좌표 반환
def fit_line(original_img, pts, fin_y=100):  #distributed_lines -> cv2.fitLine() 함수에 넣을 좌표들
    if len(pts) < 2:
        return None, None
    
    height = original_img.shape[0]

    vx, vy, x0, y0 = cv2.fitLine(np.array(pts), cv2.DIST_L2, 0, 0.01, 0.01)
    slope = safe_slope(vx, vy)

    if math.isinf(slope):
        init_x = int(x0[0])
        fin_x = int(x0[0])
    else:
        init_x = int(((height - y0[0]) / slope) + x0[0])
        fin_x = int(((fin_y - y0[0]) / slope) + x0[0])

    return slope, [(init_x, height), (fin_x, fin_y)]

# 대표 직선 검출
def regression(distributed_lines, original_img):
    right_lines, left_lines = distributed_lines
    height, width = original_img.shape[:2]
    slope = [None] * 2
    detect_code = [0, 0]
  
    # 결과 저장용
    represent_points = [None] * 4
    left_detect = 0
    right_detect = 0

    # --------오른쪽 차선 처리-------------------
    right_pts = []
    if right_lines:
        for x1, y1, x2, y2 in right_lines:
            right_pts.append((x1, y1))
            right_pts.append((x2, y2))
        
        slp, points = fit_line(original_img, right_pts)
        slope[0] = slp
        if points:
            #오른쪽 차선 하단 점
            represent_points[0] = points[0]
            #오른쪽 차선 상단 점
            represent_points[1] = points[1]
            detect_code[0] = 1
       
    # --------------- 왼쪽 차선 처리----------------
    left_pts = []
    if left_lines:
        for x1, y1, x2, y2 in left_lines:
            left_pts.append((x1, y1))
            left_pts.append((x2, y2))
        
        slp, points = fit_line(original_img, left_pts) #최소제곱법(최적 직선)을 구하는 함수
        slope[1] = slp
        if points:
            represent_points[2] = points[0]
            represent_points[3] = points[1]
            detect_code[1] = 1
            
    if all (p is None for p in represent_points):
        return None, detect_code, slope
        
    print(f"대표차선 좌표: {represent_points}")    
    return represent_points, detect_code, slope

def compute_intersection(represent_points):
    if represent_points[0] is None or represent_points[1] is  None:
        # print("오른쪽 차선 없음")
        return None
    
    elif represent_points[2] is None or represent_points[3] is None:
        # print("왼쪽 차선 없음")
        return None

    # 오른쪽 차선 기울기, y 절편 구하기
    right_fit = np.polyfit((represent_points[0][0],represent_points[1][0]),(represent_points[0][1],represent_points[1][1]), 1)
    right_slope = right_fit[0]
    right_y = right_fit[1]

    # 왼쪽
    left_fit = np.polyfit((represent_points[2][0],represent_points[3][0]),(represent_points[2][1],represent_points[3][1]), 1)
    left_slope = left_fit[0]
    left_y = left_fit[1]

    # 기울기 행렬    
    A = np.array([[right_slope, -1], [left_slope, -1]])
    # y절편 행렬                                
    B = np.array([-right_y, -left_y])
        
    # 연립방정식으로 교점 구하기
    try:
        vp = np.linalg.solve(A,B)
        print(f"{vp}")
        return tuple(vp)
            
    except np.linalg.LinAlgError:
        print("두 직선은 평행하거나 일치하여 교점을 찾을 수 없습니다.")
        return None

# 새로운 좌표계 변환 및 기하 계산 함수들 유지:
def convert_vp_to_vehicle_coords(vp, image_shape):
    img_w, img_h = image_shape[1], image_shape[0]
    x_vehicle = ((vp[0] - img_w / 2) / img_w) * 0.8
    y_vehicle = ((img_h - vp[1]) / img_h) * 1.2
    return np.array([x_vehicle, y_vehicle])

def compute_target_heading(vp_vehicle):
    target_heading = math.atan2(vp_vehicle[1], vp_vehicle[0])  # → x축 기준
    target_heading -= math.pi / 2  # → y축 기준으로 보정    
    return target_heading

def compute_current_heading(prev_steer, max_angle=math.radians(45)):
    """
    prev_steer: 실제 조향 값 (-1.23 ~ 1.0, 직진은 -0.23)
    max_angle: 기하학적 최대 조향 각도
    """
    if prev_steer >= -0.23:
        normalized = (prev_steer + 0.23) / (1.0 + 0.23)  # 우회전 영역
    else:
        normalized = (prev_steer + 0.23) / (1.23 - 0.23)  # 좌회전 영역
    heading_rad = normalized * max_angle
    return heading_rad

def compute_steering_angle(target_heading, current_heading):
    delta_rad = target_heading - current_heading
    delta_rad = (delta_rad + math.pi) % (2 * math.pi) - math.pi
    return delta_rad

def steering_angle_to_steer(delta_rad, max_angle=math.radians(45)):
    normalized = delta_rad / max_angle
    normalized = np.clip(normalized, -1.0, 1.0)
    normalized *= 0.8
    if normalized >= 0:
        return normalized * (1.0 + 0.23) - 0.23
    else:
        return normalized * (1.23 - 0.23) - 0.23


if __name__ == "__main__":
    main()