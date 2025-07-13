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
            result = regression(distributed_lines, original_img)
            
            if result[0] is None:
                self.publish_controls(-0.23, 0.0)
                return
            represent_points, detect_code, slope = result

            vp = compute_intersection(represent_points)
            if vp is None:
                self.publish_controls(-0.23, 0.0)
                return

            direction = predicDir(detect_code, slope, vp, represent_points)
            steering, throttle = compute_control(direction)
            self.publish_controls(steering, throttle)
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
    # cv2.waitKey(1)
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

    width = original_img.shape[1]
    x_center = width / 2

    if not lines:
        return [[], []]

    for i in lines:
        x1, y1, x2, y2 = i

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0:
            # 수직선: slope는 무한대 → 차선으로 쓰기 애매 → 무시
            print("수직선 (x1 == x2), 무시")
            continue

        if dy == 0:
            # 수평선 → 정지선일 가능성 높음
            print("수평선 (y1 == y2), 정지선 후보")
            continue

        slope = dy / dx

        # 오른쪽 차선
        if slope > 0 and x1 > x_center:
            right_lines.append(i)

        # 왼쪽 차선
        elif slope < 0 and x1 < x_center:
            left_lines.append(i)

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
    

def predicDir(detect_code, slope, vp, represent_points):

    if represent_points is None or any(p is None for p in represent_points):
        print("대표 점 중 None 있음 → undefined")
        return "undefined"

    if (detect_code[0] == 0) and (detect_code[1] == 0):
        print("Undefined")
        return "undefined"
    elif (detect_code[0] == 1) and (detect_code[1] == 0):
        if (slope[0] < 0):
            return "left turn"
        else:
            return "right turn"
    elif ((detect_code[0] == 0) and detect_code[1] == 1):
        if (slope[1] < 0):
            return "left turn"
        else:
            return "right turn"
        
    if (represent_points[1] is None or represent_points[3] is None):
        print("대표 점 부족 → 방향 판단 불가")
        return "undefined"
    
    thres_vp = 10.0  # 교차지점 임계값
    represent_x_center = (represent_points[1][0] + represent_points[3][0]) / 2
    print("검출한 직선 x좌표 중심", represent_x_center)
    difference = abs(slope[1]) - abs(slope[0])

    if (detect_code[1] == 1 and detect_code[0] == 1):
        if (abs(difference)) < 1:
            print("straight")
            return "straight"
        elif vp[0] < represent_x_center:
            print("left turn")
            return "left turn"
        elif vp[0] > represent_x_center:
            print("right turn")
            return "right turn"
        else:
            print("straight")
            return "straight"

    elif detect_code[1] == 1:
        print("right turn")
        return "right turn"
    elif detect_code[0] == 1:
        print("left turn")
        return "left turn"
    else:
        print("Undefined")
        return "undefined"
    
    print(f"교점과 중심 차이: {vp[0]-represent_x_center}")
               
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
    
def compute_control(direction):
    if direction == "left turn":
        steering = -0.7
        throttle = 0.2
    elif direction == "right turn":
        steering = 0.4
        throttle = 0.2
    elif direction == "straight":
        steering = -0.25
        throttle = 0.2
    else:  # undefined
        steering = 0.0
        throttle = 0.0
    return steering, throttle


if __name__ == "__main__":
    main()
