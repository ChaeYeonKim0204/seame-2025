#찐최

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

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        binary = filter_colors(original_img)
        img_mask = region_of_interest(binary)
        result = selective_erosion(img_mask)
        canny_img = apply_canny(result)
        lines = houghLines(canny_img) 

        if lines:
            distributed_lines = separateLine(lines, original_img)
            represent_points, detect_code, slope = regression(distributed_lines, original_img)
            vp = compute_intersection(represent_points)
            direction, x_offset = predicDir(detect_code, slope, vp, represent_points, original_img)
            steering, throttle = compute_control(direction, x_offset, original_img)
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
    top_width_percent = 0.5 # 상단 너비 비율
    height_percent = 0.9      # 높이 비율

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
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return canny


# 허프 변환에 새로운 함수 추가 
def houghLines(canny_img):
    lines = cv2.HoughLinesP(canny_img , rho=1, theta = np.pi/180, threshold=30, minLineLength=10, maxLineGap=5)
            
    # 직선이 감지되면
    if lines is not None:
        print("직선 검출 완료")
        
        # x1, y1, x2, y2을 반환
        return [line[0].tolist() for line in lines] 
    return []

# 오른쪽, 왼쪽 차선 분리
def separateLine(lines, original_img):
    right_lines = []
    left_lines = []

    if not lines:
        return [[], []]

    # 가중 평균 중심 계산
    x_coords = []
    weights = []
    for x1, y1, x2, y2 in lines:
        x_coords.extend([x1, x2])
        weights.extend([y1, y2])  # 아래쪽 점일수록 더 중요한 선으로 간주

    x_center = int(np.average(x_coords, weights=weights))
    print(f"[DEBUG] 동적 x_center (가중 평균): {x_center}")

    for x1, y1, x2, y2 in lines:
        if y2 - y1 == 0:
            continue  # 수평선 무시

        # 중심선을 기준으로 양쪽에 위치한 선을 분리
        if x1 < x_center and x2 < x_center:
            left_lines.append([x1, y1, x2, y2])
        elif x1 > x_center and x2 > x_center:
            right_lines.append([x1, y1, x2, y2])

    return [right_lines, left_lines]

        
#기울기와 직선의 시작점과 끝점좌표 반환
def fit_line(original_img,distributed_lines, fin_y=100):  #distributed_lines -> cv2.fitLine() 함수에 넣을 좌표들
    height = original_img.shape[0]
    if len(distributed_lines) < 2:
        return None, None

    vx, vy, x0, y0 = cv2.fitLine(np.array(distributed_lines), cv2.DIST_L2, 0, 0.01, 0.01)
    slope = vy[0] / vx[0]
    base_point = (x0[0], y0[0])

    init_x = int(((height - base_point[1]) / slope) + base_point[0])
    fin_x = int(((fin_y - base_point[1]) / slope) + base_point[0])

    return slope, [(init_x, height), (fin_x, fin_y)]

# 대표 직선 검출
def regression(distriduted_lines, original_img):
    right_lines, left_lines = distriduted_lines
    height, width = original_img.shape[:2]
    slope = [None] *2
    detect_code = [None] *2
  
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
            
        else:
            detect_code[0] = 0
       


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
         
        else:
            detect_code[1] = 0
            
    if all (p is None for p in represent_points):
        return None, detect_code, slope
        

    return represent_points, detect_code, slope

def compute_intersection(represent_points):
    if represent_points[0] is None or represent_points[1] is  None:
        print("오른쪽 차선 없음")
        return None
    
    elif represent_points[2] is None or represent_points[3] is None:
        print("왼쪽 차선 없음")
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
        return tuple(vp)
            
    except np.linalg.LinAlgError:
        print("두 직선은 평행하거나 일치하여 교점을 찾을 수 없습니다.")
        return None
    

def predicDir(detect_code, slope, vp, represent_points, canny_img):
    """
    방향 판단 및 주행 중심선 offset 계산
    """
    proc_height, proc_width = canny_img.shape[:2]
    
    if represent_points is None or all(p is None for p in represent_points):
        print("대표 점 전부 None → undefined")
        return "undefined", 0

    mid_x = proc_width // 2

    # 양쪽 차선 다 있는 경우
    if detect_code[0] == 1 and detect_code[1] == 1:
        center_line_x = int((represent_points[0][0] + represent_points[2][0]) / 2)
        x_offset = center_line_x - mid_x

        print(f"[INFO] dual lane detected, x_offset={x_offset}")

        if abs(slope[1] - slope[0]) < 1:
            return "straight", x_offset

        elif vp is not None:
            if vp[0] < center_line_x:
                return "left turn", x_offset
            else:
                return "right turn", x_offset
        else:
            if slope[0] > slope[1]:
                return "right turn", x_offset
            else:
                return "left turn", x_offset

    # 오른쪽 차선만 감지
    elif detect_code[0] == 1:
        x_offset = represent_points[0][0] - mid_x
        print(f"[INFO] only RIGHT lane detected, x_offset={x_offset}")

        if slope[0] is not None and slope[0] > 0.7:
            return "right turn", x_offset
        else:
            return "straight", x_offset

    # 왼쪽 차선만 감지
    elif detect_code[1] == 1:
        x_offset = represent_points[2][0] - mid_x
        print(f"[INFO] only LEFT lane detected, x_offset={x_offset}")

        if slope[1] is not None and slope[1] < -0.7:
            return "left turn", x_offset
        else:
            return "straight", x_offset

    # 대표 좌표 일부만 있고 usable하지 않은 상황
    print("대표 좌표 일부 누락 → fallback to undefined")
    return "straight", 0  # fallback
 
def draw_detected_lines(original_img, represent_points, vp=None):
    # 복사본 만들기
    img = original_img.copy()

    # 대표 차선 좌표가 있을 때만 그림
    # 오른쪽 차선 (파란색)
    if represent_points[0] and represent_points[1]:
        cv2.line(img, represent_points[0], represent_points[1], (255, 0, 0), 3)

    # 왼쪽 차선 (초록색)
    if represent_points[2] and represent_points[3]:
        cv2.line(img, represent_points[2], represent_points[3], (0, 255, 0), 3)

    # 교차점 (빨간 점)
    if vp is not None:
        vp_point = (int(vp[0]), int(vp[1]))
        cv2.circle(img, vp_point, 8, (0, 0, 255), -1)

    return img
    
def compute_control(direction, x_offset, canny_img):
    """
    중심선 offset을 바탕으로 steering 계산
    """
    if direction == "undefined":
        return 0.0, 0.0  # 정지
   
    proc_height, proc_width = canny_img.shape[:2]

    y_offset = proc_height // 2
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_deg = angle_to_mid_radian * 180.0 / math.pi
    steering_angle = angle_deg / 120  # 정규화된 steering 값 (-1 ~ 1 사이)

    steering = float(np.clip(steering_angle, -1.0, 1.0))  # 안전 범위로 제한
    throttle = 0.2 if direction != "undefined" else 0.0

    print(f"[CONTROL] direction: {direction}, steering: {steering:.2f}, throttle: {throttle}")
    return steering, throttle



if __name__ == "__main__":
    main()

    # # --- 이미지 파일 테스트 추가 ---
    # img = cv2.imread("C:/line6.jpg")
    # scale_percent = 50
    # width = int(img.shape[1] * scale_percent / 100)
    # height = int(img.shape[0] * scale_percent / 100)
    # dim = (width, height)
    # original_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    # binary = filter_colors(original_img)
    # img_mask = region_of_interest(binary)
    # result = selective_erosion(img_mask)
    # canny_img = apply_canny(result)
    # lines = houghLines(canny_img)

    # if lines:
    #     distributed_lines = separateLine(lines, original_img)
    #     represent_points, detect_code, slope = regression(distributed_lines, original_img)
    #     vp = compute_intersection(represent_points)
    #     direction = predicDir(detect_code, slope, vp, represent_points)  # ⬅️ 여기에 방향 변수 반영
    #     print("Direction:", direction)
    #     result_img = draw_detected_lines(original_img, represent_points, vp)
    #     cv2.imshow("Detected Lines", result_img)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    # else:
    #     print("No lines detected")