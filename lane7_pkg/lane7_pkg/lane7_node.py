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
        self.throttle = 0.2  # 초기값
        
        # LaneCenterDetector 인스턴스를 한 번만 생성
        self.detector = LaneCenterDetector(slice_y=120, height=15, method='histogram')

    def callback(self, msg):
        try:
            original_img = self.cb.imgmsg_to_cv2(msg, "bgr8")
            
            # 중심 x 좌표 추출
            center_x = self.detector.run(original_img)

            if center_x is not None:
                # PID 기반 방향 예측 및 제어값 계산
                self.steering, self.throttle = self.predicDir(center_x)
            else:
                self.steering, self.throttle = 0.0, 0.0
                print("[INFO] 차선을 찾지 못함 - 정지")

            # 퍼블리시
            self.publish_controls(self.steering, self.throttle)
            
        except Exception as e:
            print(f"[ERROR] Callback error: {e}")
            self.publish_controls(0.0, 0.0)

    def publish_controls(self, steering, throttle):
        msg_s = Float32()
        msg_s.data = float(np.clip(steering, -0.7, 0.7))
        print(f"[PUBLISH] Steering: {msg_s.data:.3f}")
        self.steering_pub.publish(msg_s)

        msg_t = Float32()
        msg_t.data = float(np.clip(throttle, 0.0, 0.5))
        self.throttle_pub.publish(msg_t)

    def predicDir(self, center_x):
        """PID 제어 함수 - 변수명 수정"""
        if center_x is None:
            return 0.0, 0.0

        if self.target_pixel is None:
            self.target_pixel = center_x
            print(f"[INFO] Automatically chosen line position = {self.target_pixel}")

        if self.pid_st.setpoint != self.target_pixel:
            self.pid_st.setpoint = self.target_pixel

        steering = self.pid_st(center_x)
        steering -= 0.24

        if abs(center_x - self.target_pixel) > self.cfg['target_threshold']:
            if self.throttle > self.cfg['throttle_min']:
                self.throttle -= self.cfg['delta_th']
            if self.throttle < self.cfg['throttle_min']:
                self.throttle = self.cfg['throttle_min']
        else:
            if self.throttle < self.cfg['throttle_max']:
                self.throttle += self.cfg['delta_th']
            if self.throttle > self.cfg['throttle_max']:
                self.throttle = self.cfg['throttle_max']

        print(f"[PID CONTROL] steering: {steering:.3f}, throttle: {self.throttle:.3f}, center_x: {center_x}, target: {self.target_pixel}")
        return steering, self.throttle
        
def main():
    rp.init()
    pid = PID(0.45, 0.0007, 0.15, setpoint=0)
    cfg = {
        'PID_P': 0.45,
        'PID_I': 0.0007,
        'PID_D': 0.15,
        'target_threshold': 20,
        'throttle_min': 0.2,
        'throttle_max': 0.3,
        'delta_th': 0.02
    }
    image_subscriber = ImageSubscriber(pid, cfg)
    
    rp.spin(image_subscriber)
    image_subscriber.destroy_node()
    rp.shutdown()

def find_lane_center_improved(image, slice_y=120, height=10, min_gap=30):
    """
    두 개의 흰색 차선을 정확히 인식하고 중앙 x좌표를 반환
    
    Args:
        image: 입력 이미지
        slice_y: 스캔할 y 위치
        height: 스캔 영역 높이  
        min_gap: 두 차선 사이 최소 간격 (픽셀)
    """
    # 1. HSV 변환 및 흰색 마스킹
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 220], dtype=np.uint8)
    upper_white = np.array([180, 30, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # 2. 관심 영역 자르기 (slice)
    roi = mask[slice_y:slice_y + height, :]
    
    # 3. 각 행에서 흰색 픽셀 그룹 찾기
    lane_groups = []
    
    for row in range(height):
        white_pixels = np.where(roi[row] == 255)[0]
        if len(white_pixels) == 0:
            continue
            
        # 연속된 픽셀들을 그룹화
        groups = []
        current_group = [white_pixels[0]]
        
        for i in range(1, len(white_pixels)):
            if white_pixels[i] - white_pixels[i-1] <= 2:  # 연속성 허용 갭
                current_group.append(white_pixels[i])
            else:
                if len(current_group) > 2:  # 최소 그룹 크기
                    groups.append(current_group)
                current_group = [white_pixels[i]]
        
        # 마지막 그룹 추가
        if len(current_group) > 5:
            groups.append(current_group)
        
        lane_groups.extend(groups)
    
    if not lane_groups:
        print("[WARN] 차선 그룹이 감지되지 않음")
        return None, roi
    
    # 4. 그룹들의 중심점 계산
    group_centers = []
    for group in lane_groups:
        center_x = (min(group) + max(group)) / 2
        group_centers.append(center_x)
    
    # 5. 클러스터링으로 두 차선 구분
    group_centers = np.array(group_centers)
    group_centers_sorted = np.sort(group_centers)
    
    # 두 차선 찾기
    lanes = []
    current_lane = [group_centers_sorted[0]]
    
    for i in range(1, len(group_centers_sorted)):
        if group_centers_sorted[i] - group_centers_sorted[i-1] <= min_gap:
            current_lane.append(group_centers_sorted[i])
        else:
            if len(current_lane) > 1:  # 최소 포인트 수
                lanes.append(current_lane)
            current_lane = [group_centers_sorted[i]]
    
    # 마지막 차선 추가
    if len(current_lane) > 1:
        lanes.append(current_lane)
    
    if len(lanes) < 2:
        print(f"[WARN] 차선 2개가 감지되지 않음 (감지된 차선: {len(lanes)})")
        return None, roi
    
    # 6. 각 차선의 평균 위치 계산
    left_lane_center = np.mean(lanes[0])
    right_lane_center = np.mean(lanes[-1])  # 가장 오른쪽 차선
    
    # 7. 두 차선의 중심점
    center_x = int((left_lane_center + right_lane_center) / 2)
    
    return center_x, roi

def find_lane_center_histogram(image, slice_y=120, height=10, min_separation=50):
    """
    히스토그램 기반 두 차선 중심 찾기 (더 간단한 방법)
    scipy 없이 구현
    """
    # 1. HSV 변환 및 흰색 마스킹
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 220], dtype=np.uint8)
    upper_white = np.array([180, 30, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # 2. 관심 영역 자르기
    roi = mask[slice_y:slice_y + height, :]
    
    # 3. 수직 히스토그램 생성
    histogram = np.sum(roi, axis=0)
    
    # 4. 간단한 피크 찾기 (scipy 없이)
    threshold = height * 30  # 임계값
    peaks = []
    
    for i in range(min_separation, len(histogram) - min_separation):
        if histogram[i] > threshold:
            # 주변보다 큰지 확인
            is_peak = True
            for j in range(i - min_separation//2, i + min_separation//2 + 1):
                if j != i and histogram[j] >= histogram[i]:
                    is_peak = False
                    break
            if is_peak:
                peaks.append(i)
    
    if len(peaks) < 2:
        print(f"[WARN] 충분한 피크가 감지되지 않음 (감지된 피크: {len(peaks)})")
        return None, roi
    
    # 5. 가장 강한 두 피크 선택
    peak_heights = [histogram[p] for p in peaks]
    sorted_indices = np.argsort(peak_heights)[::-1]
    
    # 상위 2개 피크 선택
    top_peaks = [peaks[sorted_indices[0]], peaks[sorted_indices[1]]]
    top_peaks = sorted(top_peaks)  # 왼쪽부터 정렬
    
    # 6. 중심점 계산
    left_peak = top_peaks[0]
    right_peak = top_peaks[1]
    center_x = (left_peak + right_peak) // 2
    
    return center_x, roi

# Donkeycar 스타일 Part 클래스
class LaneCenterDetector:
    """
    Donkeycar Part 스타일의 라인 중심 감지기
    """
    def __init__(self, slice_y=120, height=10, method='histogram'):
        self.slice_y = slice_y
        self.height = height
        self.method = method
        
    def run(self, image):
        """
        Donkeycar Part 인터페이스
        
        Args:
            image: 카메라 이미지
            
        Returns:
            lane_center: 차선 중심 x 좌표 (None if 실패)
        """
        try:
            if self.method == 'histogram':
                center_x, vis = find_lane_center_histogram(image, self.slice_y, self.height)
            else:
                center_x, vis = find_lane_center_improved(image, self.slice_y, self.height)
                
            return center_x
        except Exception as e:
            print(f"[ERROR] Lane detection error: {e}")
            return None

# 사용 예시
if __name__ == "__main__":
    main()