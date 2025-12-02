import cv2
import numpy as np
from picamera2 import Picamera2

def nothing(x):
    pass

# --- 初始化相机 (RPi 5 专用) ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
picam2.configure(config)
picam2.start()
# -----------------------------

cv2.namedWindow("Trackbars")

# 创建滑块 (默认值设为黄色范围)
cv2.createTrackbar("L - H", "Trackbars", 20, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 100, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 40, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

print("正在运行... 按 'q' 退出并查看结果。")

try:
    while True:
        # 获取图像
        frame = picam2.capture_array()
        
        # 转换颜色空间 BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 读取滑块位置
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        
        lower_color = np.array([l_h, l_s, l_v])
        upper_color = np.array([u_h, u_s, u_v])
        
        # 创建遮罩
        mask = cv2.inRange(hsv, lower_color, upper_color)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # 显示
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("\n" + "="*30)
            print("记录这些数值:")
            print(f"lower_yellow = np.array([{l_h}, {l_s}, {l_v}])")
            print(f"upper_yellow = np.array([{u_h}, {u_s}, {u_v}])")
            print("="*30 + "\n")
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
