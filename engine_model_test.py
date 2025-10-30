import cv2
from ultralytics import YOLO

# --- 설정 ---
ENGINE_MODEL_PATH = '' # 모델 경로 업로드 (.engine 파일 업로드)

def main():
    # 1. YOLO TensorRT 엔진 모델 로드
    print(f"'{ENGINE_MODEL_PATH}' 모델을 로드합니다.")
    try:
        model = YOLO(ENGINE_MODEL_PATH, task='detect') # task는 detect로 고정!
    except Exception as e:
        print(f"모델 로드 중 오류 발생: {e}")
        return

    # 2. USB 카메라 실행 (V4L2 모드)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 해상도 설정
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)            # FPS 설정

    if not cap.isOpened():
        print("USB 카메라를 열 수 없습니다. 장치 인덱스(/dev/video*) 확인 필요")
        return

    print("USB 카메라가 준비되었습니다. 실시간 탐지를 시작합니다. (종료하려면 'q'를 누르세요)")

    # 3. 실시간 탐지 루프
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # YOLO 추론
        results = model(frame, verbose=False)

        # 바운딩 박스 시각화
        annotated_frame = results[0].plot()

        # 화면 출력
        cv2.imshow("YOLO Detection (USB Webcam V4L2)", annotated_frame)

        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4. 자원 해제
    print("프로그램을 종료합니다.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
