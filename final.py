import cv2
from ultralytics import YOLO

# 1. YOLO 엔진 모델 로드 (.engine 파일)
model = YOLO("crack.engine", task='detect')

# 2. GStreamer 파이프라인 (USB 웹캠, MJPEG 기반)
gst_str = (
    "v4l2src device=/dev/video1 ! "
    "image/jpeg, width=640, height=480, framerate=30/1 ! "
    "jpegdec ! videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true sync=false"
)

# 3. 카메라 열기
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("카메라를 열 수 없습니다. 파이프라인을 확인하세요.")
    exit()

print("카메라 연결 성공. YOLO 실시간 탐지 시작 (종료: q)")

# 4. 루프 실행
while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임 읽기 실패")
        break

    # YOLO 추론
    results = model(frame, verbose=False)

    # 탐지 결과 시각화
    annotated_frame = results[0].plot()

    # 화면 출력
    cv2.imshow("YOLO Detection (MJPEG)", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 5. 종료 처리
print("프로그램 종료")
cap.release()
cv2.destroyAllWindows()
