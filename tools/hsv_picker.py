# Click a pixel to print its HSV value
# Shared by Sihyeon Park on 2025-07-18 11:06. A variant of this picker was used
# that morning to measure the red child-zone and yellow-line colours; the red
# thresholds in lane_following_node.py come from those measurements.
# The original snippet assumed `img` and `hsv` were already loaded; the
# loading lines below were added so it runs on its own.
import sys

import cv2

if len(sys.argv) < 2:
    sys.exit("usage: python3 hsv_picker.py <image>")
img = cv2.imread(sys.argv[1])
if img is None:
    sys.exit(f"cannot read {sys.argv[1]}")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 마우스 클릭 시 HSV 출력 함수
def show_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = hsv[y, x]
        print(f"HSV at ({x},{y}): H={pixel[0]}, S={pixel[1]}, V={pixel[2]}")

# 창 띄우기 및 콜백 연결
cv2.imshow("Image", img)
cv2.setMouseCallback("Image", show_hsv)

cv2.waitKey(0)
cv2.destroyAllWindows()
