import time
import cv2
import numpy as np
# 导入树莓派5专用的相机库
from picamera2 import Picamera2

print("正在初始化 Picamera2 (RPi 5)...")

# 1. 初始化相机
picam2 = Picamera2()

# 2. 配置相机
# size: 分辨率 (640x480 对树莓派负担小，帧率高)
# format: BGR888 是 OpenCV 默认的颜色格式
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
picam2.configure(config)

# 3. 启动相机
picam2.start()

print("相机已启动。按 'q' 键退出。")

try:
    while True:
        # 4. 直接获取图像数组 (这比 cv2.VideoCapture 快得多且不报错)
        frame = picam2.capture_array()

        # --- 下面是画十字准星的逻辑 (和之前一样) ---
        height, width = frame.shape[:2]
        cx, cy = int(width / 2), int(height / 2)
        
        # 画红色十字
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)
        # ---------------------------------------

        cv2.imshow("RPi 5 Focus Tool", frame)

        # 按 q 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"发生错误: {e}")

finally:
    # 清理资源
    picam2.stop()
    cv2.destroyAllWindows()
