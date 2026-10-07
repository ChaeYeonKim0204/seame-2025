# Live USB-camera preview; press s to save capture.png, Esc to quit
# Shared by Yoonju Jeong on 2025-07-17 04:37, after the bird's-eye-view reference photos. Kept as written.

import cv2

# ✅ 1. USB 웹캠 열기 (보통 장치 번호는 0, 여러 개면 1, 2로 바뀔 수 있음)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # V4L2는 리눅스에서 안정적으로 작동

if not cap.isOpened():
    print("[ERROR] USB 카메라를 열 수 없습니다.")
    exit()

# ✅ 2. 해상도 설정 (1280x720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# ✅ 3. 실제 설정된 해상도 확인 (디버깅용)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"[INFO] 실제 카메라 해상도: {int(width)} x {int(height)}")

# ✅ 4. 루프 돌면서 실시간 영상 표시 및 저장
while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 프레임을 읽지 못했습니다.")
        break

    cv2.imshow("Live", frame)

    key = cv2.waitKey(1)

    if key == ord('s'):  # s 키 누르면 이미지 저장
        cv2.imwrite('capture.png', frame)
        print("[INFO] 이미지 저장 완료: capture.png")

    if key == 27:  # ESC 키로 종료
        print("[INFO] 종료합니다.")
        break

# ✅ 5. 정리
cap.release()
cv2.destroyAllWindows()
