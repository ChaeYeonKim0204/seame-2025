import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.callback,
            qos_profile_sensor_data 
        )
        self.cb = CvBridge()

    def callback(self, msg):
        original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
        
        # 아래 함수들은 미리 정의돼 있어야 합니다.
        binary = detect_stop_line(original_img)
        img_mask = region_of_interest(binary)
        lines = houghLines(img_mask)
        # distributed_lines = separateLine(lines,original_img)
        # represent_point, detect_code, slope = regression(distributed_lines, original_img)
        # vp = compute_intersection(represent_point)
        # predicDir(detect_code, slope, vp)

        if lines is not None and len(lines) > 0:
            self.get_logger().info(f"검출된 직선 개수: {len(lines)}")
            for line in lines:
                self.get_logger().info(f"직선: {line}")
        else:
            self.get_logger().info("직선 없음")

        # 이미지 시각화 (선택)
        cv2.imshow("original", original_img)
        cv2.imshow("binary", binary)
        cv2.imshow("roi", img_mask)
        cv2.waitKey(1)

def main():
    rp.init()
    image_subscriber = ImageSubscriber()
    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown()

class JdOpencvLaneDetect(object):  # 차선 인식 클래스 정의
    def __init__(self):
        self.curr_steering_angle = 90  # 현재 조향각, 90은 직진 의미

def detect_stop_line(original_img):
    # 흑백 변환
    gray = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러로 노이즈 제거
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # 이진화
    _, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

    # 윤곽선 검출
    #contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return binary

def region_of_interest(binary):  #cv2.Canny()를 통해 엣지 검출이 된 흑백 이미지
    height, width = binary.shape
    mask = np.zeros_like(binary)  #빈 마스크 생성 (검정색만 있는 이미지)

    # 화면 아래 절반만 관심 영역으로 설정
    polygon = np.array([[
        (0, height * 0.5),
        (width, height * 0.5),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)  # 관심 영역을 흰색(255)으로 채움
    show_image("mask", mask)

    img_mask = cv2.bitwise_and(binary, mask)  # 관심영역만 추출
    return img_mask

def show_image(winname, img):
    cv2.imshow(winname, img)
    cv2.waitKey(1)

#관심영역 설정한 이미지
def houghLines(img_mask):
    lines = cv2.HoughLinesP(img_mask , rho=1, theta=np.pi/180, threshold=20, minLineLength=10, maxLineGap=20)

    if lines is not None:
        print("직선검출")
        return [line[0].tolist() for line in lines]  # x1, y1, x2, y2
    # return []


#허프변환한 직선들
def separateLine(lines):
    right_lines = []
    left_lines = []
    
    if not lines:
        return [[],[]]
        
    slope_thresh = 0.3 #기울기 임계값(수정)
    print("직선 검출 성공")                
    
    for i in lines:  # lines: houghLine에서 추출한 
        x1,y1,x2,y2 = i
        
        if (x2 - x1) == 0:
            print("정지선")
            
        fit = np.polyfit((x1,x2),(y1,y2),1) #좌표주고 1차방정식 구함(기울기,y절편)
        slope = fit[0]
        y_intercept = fit[1]
        
        if abs(slope) > slope_thresh:
            if slope < 0 and x1 > x_center:
                right_lines.append(i)
                right_detect = 1
            elif slope > 0 and x1 < x_center:
                left_lines.append(i)
                left_detect = 1
    for i in right_lines:
        print(f"오른쪽 차선: {i}")

    for i in left_lines:
        print(f"왼쪽차선: {i}")			
    return [right_lines, left_lines]

if __name__ == "__main__":
	main()

