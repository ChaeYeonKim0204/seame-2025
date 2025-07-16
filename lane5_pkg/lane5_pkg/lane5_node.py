import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math
from simple_pid import PID

class ImageSubscriber(Node):
    def __init__(self, pid, cfg):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/image', self.callback, qos_profile_sensor_data)
        self.steering_pub = self.create_publisher(Float32, '/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.cb = CvBridge()
        self.pid_st = pid 
        self.cfg = cfg
        self.target_pixel = None
        self.throttle = 0.2 # 초기값
 

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        binary = filter_colors(original_img)
        img_mask = region_of_interest(binary)
        result = selective_erosion(img_mask)
        canny_img = apply_canny(result)
        lines = houghLines(canny_img)
        image_height, image_width = original_img.shape[:2] 

        if lines:
            distributed_lines = separateLine(lines, original_img)
            result = regression(distributed_lines, original_img)

            if result is not None:
                if result[0] is None:
                    self.publish_controls(-0.24, 0.0)
                    return
            else:
                return

            represent_points, detect_code, slope = result

            center_fitx = compute_intersection(represent_points, original_img)
            # predic_result = predicDir(center_fitx, represent_points)
        
            if center_fitx is not None:
                predic_result = predicDir(self, center_fitx, represent_points)
                if predic_result is None:
                    self.steering, self.throttle = 0.0, 0.0
                else:
                    self.steering, self.throttle = predic_result
            else:
                self.steering, self.throttle = 0.0, 0.0

            self.publish_controls(self.steering, self.throttle)
        else:
            self.publish_controls(0.0, 0.0)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -0.7, 0.7))
        print(f"[PUBLISH] Steering: {msg_s.data:.3f}")
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = float(np.clip(self.throttle, 0.0, 0.5))
        self.throttle_pub.publish(msg_t)
        
def main():
    rp.init()
    pid = PID(0.00002, 0.001, 0.01, setpoint=0)
    cfg = {
        'PID_P': 0.00002,
        'PID_I': 0.001,
        'PID_D': 0.01,
        'target_threshold': 20,
        'throttle_min': 0.2,
        'throttle_max': 0.3,
        'delta_th': 0.02
    }
    image_subscriber = ImageSubscriber(pid, cfg)
    
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
        # print("직선 검출 완료")
        
        # x1, y1, x2, y2을 반환
        return [line[0].tolist() for line in lines] 
    return []

# 오른쪽, 왼쪽 차선 분리
def separateLine(lines, original_img):
    right_lines = []
    left_lines = []

    if not lines:
        return [[], []]

    x_center = original_img.shape[1] /2
   
        # 중심선을 기준으로 양쪽에 위치한 선을 분리
    for x1, y1, x2, y2 in lines:
        if x1 < x_center and x2 < x_center:
            left_lines.append([x1, y1, x2, y2])
        elif x1 > x_center and x2 > x_center:
            right_lines.append([x1, y1, x2, y2])
    
    # print(f"[right_lines: {right_lines}, left_lines: {left_lines}]")      
    return [right_lines, left_lines]
    
def safe_slope(vx, vy, epsilon=1e-6):
    if abs(vx[0]) < epsilon:
        # 수직선 또는 수직에 가까운 선
        return float('inf')
    else:
        return vy[0] / vx[0]

        
#기울기와 직선의 시작점과 끝점좌표 반환
def fit_line(original_img, distributed_lines, fin_y=300):  # distributed_lines -> cv2.fitLine() 함수에 넣을 좌표들
    height = original_img.shape[0]
    if len(distributed_lines) < 2:
        return None, None

    vx, vy, x0, y0 = cv2.fitLine(np.array(distributed_lines), cv2.DIST_L2, 0, 0.01, 0.01)
    slope = vy[0] / vx[0]
    base_point = (x0[0], y0[0])

    if math.isinf(slope):
        init_x = int(x0[0])
        fin_x = int(x0[0])
    else:
        init_x = int(((height - y0[0]) / slope) + x0[0])
        fin_x = int(((fin_y - y0[0]) / slope) + x0[0])

    middle_y = 150
    middle_x = int(((middle_y - base_point[1]) / slope) + base_point[0])

    return slope, [(init_x, height), (fin_x, fin_y), (middle_x, middle_y)]

# 대표 직선 검출
def regression(distriduted_lines, original_img):
    right_lines, left_lines = distriduted_lines
    height, width = original_img.shape[:2]
    slope = [None] *2
    detect_code = [None] *2
  
    # 결과 저장용
    represent_points = [None] * 6

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
            #오른쪽 중간점
            represent_points[2] = points[2]
            
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
            represent_points[3] = points[0]
            represent_points[4] = points[1]
            represent_points[5] = points[2]

            detect_code[1] = 1
         
        else:
            detect_code[1] = 0            
    if all (p is None for p in represent_points):
        return None, detect_code, slope
        

    return represent_points, detect_code, slope

def compute_intersection(represent_points, original_img):
    height, width = original_img.shape[:2]
    if represent_points[0] is None or represent_points[1] is None or represent_points[2] is None:
        print("오른쪽 차선 없음")
        return None
    
    # 왼쪽 차선 체크 (3, 4, 5)
    elif represent_points[3] is None or represent_points[4] is None or represent_points[5] is None:
        print("왼쪽 차선 없음")
        return None
    
    # 오른쪽 차선 기울기, y 절편 구하기
    right_fit = np.polyfit((represent_points[0][1],represent_points[1][1],represent_points[2][1]),(represent_points[0][0],represent_points[1][0],represent_points[2][0]), 2)
    # 왼쪽
    left_fit = np.polyfit((represent_points[3][1],represent_points[4][1],represent_points[5][1]),(represent_points[3][0],represent_points[4][0],represent_points[5][0]), 2)
    # 기울기 행렬    
    ploty = np.linspace(0,height-1, height)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    center_fitx_array = (left_fitx + right_fitx) / 2
    
    target_y = int(height * 0.6)
    target_x = int(center_fitx_array[target_y])
    center_fitx = target_x
    
    # # 🖼️ 시각화
    # vis_img = original_img.copy()

    # # 좌우 차선 선 그리기 (파란색)
    # for i in range(0, height - 1, 5):
    #     pt1 = (int(left_fitx[i]), int(ploty[i]))
    #     pt2 = (int(left_fitx[i + 1]), int(ploty[i + 1]))
    #     if 0 <= pt1[0] < width and 0 <= pt2[0] < width:
    #         cv2.line(vis_img, pt1, pt2, (255, 0, 0), 2)

    #     pt1 = (int(right_fitx[i]), int(ploty[i]))
    #     pt2 = (int(right_fitx[i + 1]), int(ploty[i + 1]))
    #     if 0 <= pt1[0] < width and 0 <= pt2[0] < width:
    #         cv2.line(vis_img, pt1, pt2, (255, 0, 0), 2)

    # # 중앙선 그리기 (초록색)
    # for i in range(0, height - 1, 5):
    #     pt1 = (int(center_fitx_array[i]), int(ploty[i]))
    #     pt2 = (int(center_fitx_array[i + 1]), int(ploty[i + 1]))
    #     if 0 <= pt1[0] < width and 0 <= pt2[0] < width:
    #         cv2.line(vis_img, pt1, pt2, (0, 255, 0), 2)

    # # 목표 포인트 (빨간 점)
    # cv2.circle(vis_img, (target_x, target_y), 6, (0, 0, 255), -1)

    # # 텍스트 출력
    # cv2.putText(vis_img, f"Target X: {target_x}", (10, 30),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # # 실제 표시
    # cv2.imshow("Lane + Center Line", vis_img)
    # cv2.waitKey(1)

    return center_fitx


def predicDir(self, center_fitx, represent_points):
    if center_fitx is None:
        return 0.0, 0.0

    if self.target_pixel is None:
        self.target_pixel = center_fitx
        print(f"[INFO] Automatically chosen line position = {self.target_pixel}")

    if self.pid_st.setpoint != self.target_pixel:
        self.pid_st.setpoint = self.target_pixel

    steering = self.pid_st(center_fitx)
    steering -= 0.24
    

    if abs(center_fitx - self.target_pixel) > self.cfg['target_threshold']:
        if self.throttle > self.cfg['throttle_min']:
            self.throttle -= self.cfg['delta_th']
        if self.throttle < self.cfg['throttle_min']:
            self.throttle = self.cfg['throttle_min']
    else:
        if self.throttle < self.cfg['throttle_max']:
            self.throttle += self.cfg['delta_th']
        if self.throttle > self.cfg['throttle_max']:
            self.throttle = self.cfg['throttle_max']

    print(f"[PID CONTROL] steering: {steering:.3f}, throttle: {self.throttle:.3f}, center_fitx: {center_fitx}, target: {self.target_pixel}")
    return steering, self.throttle

def draw_detected_lines(original_img, represent_points, vp=None):
    # 복사본 만들기
    img = original_img.copy()

    # 대표 차선 좌표가 있을 때만 그림
    # 오른쪽 차선 (파란색)
    if represent_points[0] is None or represent_points[1] is None or represent_points[2] is None:
            print("오른쪽 차선 없음")
            return None
        
    # 왼쪽 차선 점들 (3, 4, 5)
    elif represent_points[3] is None or represent_points[4] is None or represent_points[5] is None:
        print("왼쪽 차선 없음")
        return None
        
    # 교차점 (빨간 점)
    if vp is not None:
        vp_point = (int(vp[0]), int(vp[1]))
        cv2.circle(img, vp_point, 8, (0, 0, 255), -1)

    return img
    



if __name__ == "__main__":
    main()

