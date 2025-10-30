import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# ✅ 창 크기를 자유롭게 변경할 수 있도록 설정
cv2.namedWindow("USB Camera Test", cv2.WINDOW_NORMAL)

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # ✅ 창 크기 조정 (예: 가로 1280, 세로 720)
    cv2.resizeWindow("USB Camera Test", 1280, 720)

    # ✅ 원하면 프레임 자체를 리사이즈해서 표시할 수도 있음
    # frame = cv2.resize(frame, (1280, 720))

    cv2.imshow("USB Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
