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

        # cv2.imshow("original", original_img)
        # cv2.imshow("binary", binary)
        cv2.imshow("roi", img_mask)
        # cv2.waitKey(1)

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

def filter_colors(original_img):
    hsv = cv2.cvtColor(original_img,cv2.COLOR_BGR2HSV)
    
    #흰색 임계값(값 수정 필요)
    lower_white = np.array([0,0,200], dtype=np.uint8)
    upper_white = np.array([180,30,255], dtype = np.uint8)
    binary = cv2.inRange(hsv, lower_white, upper_white)
    
    cv2.imshow("Binary Image", binary) # "Binary Image"는 창의 제목입니다.
    cv2.waitKey(0)# 키보드 입력 대기. 0은 아무 키나 누를 때까지 무한 대기.
    cv2.destroyAllWindows() # 모든 OpenCV 창 닫기
    
    return binary

def detect_stop_line(original_img):
    gray = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    return binary

#사다리꼴로 관심영역 지정
def region_of_interest(binary):  
    height, width = binary.shape[:2]
    mask = np.zeros_like(binary)  #빈 마스크 생성 (검정색만 있는 이미지)
     
    bottom_width_percent = 0.9  # 하단 너비 비율
    top_width_percent = 0.6 # 상단 너비 비율
    height_percent = 0.56      # 높이 비율

    bottom_left = (int(width * (0.5 - bottom_width_percent / 2)), height)
    bottom_right = (int(width * (0.5 + bottom_width_percent / 2)), height)
    top_left = (int(width * (0.5 - top_width_percent / 2)), int(height * (1 - height_percent)))
    top_right = (int(width * (0.5 + top_width_percent / 2)), int(height * (1 - height_percent)))

    polygon = np.array([[bottom_left, top_left, top_right, bottom_right]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    img_mask = cv2.bitwise_and(binary, mask)

    cv2.imshow("roi Image", img_mask) 
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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

def houghLines(img_mask):
    lines = cv2.HoughLinesP(img_mask , rho=1, theta = np.pi/180, threshold=30, minLineLength=10, maxLineGap=5)
            
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

    width = original_img.shape[1]
    x_center = width / 2

    slope_thresh = 0.3

    if not lines:
         return [[],[]]
    for i in lines:
        x1,y1,x2,y2 = i

        if x2 - x1 == 0:
            print("정지선")

        fit = np.polyfit((x1,x2),(y1,y2),1)
        slope = fit[0]
        y_intercept = fit[0] # Gpt가 fit[1]이 맞다는디
        
        # 오른쪽 차선 판별
        if slope > 0 and x1 > x_center:
            right_lines.append(i)
            right_detect = 1

        # 왼쪽 차선 판별
        elif slope < 0 and x1< x_center:
            left_lines.append(i)
            left_detect = 1

    return [right_lines, left_lines]

#기울기와 직선의 시작점과 끝점좌표 반환
def fit_line(original_img,distributed_lines, fin_y=300):  #distributed_lines -> cv2.fitLine() 함수에 넣을 좌표들
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

def predicDir(detect_code, slope, vp, original_img): #detect_code, slope, vp

    if (detect_code[1] ==0) and (detect_code[0] ==0):
        print("Undefined")

    thres_vp = 10.0 # 교차지점 임계값
    width = original_img.shape[1]
    img_center = width /2

    if (detect_code[1] == 1 and  detect_code[0] == 1):
        if (slope[1] - abs(slope[0])) == 0.5:
            print("straight")
            
        if vp[0] < img_center:
            print("left turn")
            
        elif vp[0] > img_center:
            print("right turn")
            
        else:
            print("straight")

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