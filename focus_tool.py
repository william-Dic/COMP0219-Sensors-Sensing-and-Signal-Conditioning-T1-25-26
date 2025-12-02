import cv2

# 初始化摄像头
cap = cv2.VideoCapture(0)

# 设置分辨率 (建议设置为高一点，方便看清细节)
cap.set(3, 1280)
cap.set(4, 720)

print("正在运行调焦助手...")
print("请旋转镜头调整焦距。")
print("按 'q' 键退出。")

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头")
        break

    # 获取画面中心点
    height, width = frame.shape[:2]
    cx, cy = int(width / 2), int(height / 2)

    # 画一个十字准星，方便对焦观察
    # 水平线
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
    # 垂直线
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

    cv2.imshow("Focus Adjustment", frame)

    # 按 q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
