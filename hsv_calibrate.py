import cv2
import numpy as np

def nothing(x):
    pass

# 初始化摄像头
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# 创建调节窗口
cv2.namedWindow("Trackbars")

# 创建6个滑块
# 初始值我设置为了黄色的近似范围，方便你开始调节
cv2.createTrackbar("L - H", "Trackbars", 20, 179, nothing) # 色调下限
cv2.createTrackbar("L - S", "Trackbars", 100, 255, nothing) # 饱和度下限
cv2.createTrackbar("L - V", "Trackbars", 100, 255, nothing) # 亮度下限
cv2.createTrackbar("U - H", "Trackbars", 40, 179, nothing) # 色调上限
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing) # 饱和度上限
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing) # 亮度上限

print("调节滑块，直到 'Mask' 窗口中只有你的黄色物体是白色。")
print("完成后按 'q' 退出，程序会输出最终数值。")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 转换到 HSV 空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 获取滑块当前位置
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
    
    # 生成预览结果 (黄色物体保留原色，背景变黑)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # 显示窗口
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        print("\n" + "="*30)
        print("请将以下数值复制到你的测量代码中：")
        print(f"lower_yellow = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"upper_yellow = np.array([{u_h}, {u_s}, {u_v}])")
        print("="*30 + "\n")
        break

cap.release()
cv2.destroyAllWindows()
