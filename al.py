import cv2
from ultralytics import YOLO
import simpleaudio as sa  # simpleaudio 라이브러리 추가

# --- 설정 ---
ENGINE_MODEL_PATH = 'fire_s.engine'
WAV_PATH = 'fire_cut.wav'  # 알림으로 사용할 .wav 파일 경로
TARGET_CLASS = 'fire'  # 알림을 울릴 특정 클래스 이름

def main():
    # 1. 알림음 파일 로드
    try:
        wave_obj = sa.WaveObject.from_wave_file(WAV_PATH)
        play_obj = None  # 사운드 재생 상태를 추적할 변수
    except FileNotFoundError:
        print(f"경고: 알림음 파일 '{WAV_PATH}'를 찾을 수 없습니다.")
        wave_obj = None
        play_obj = None

    # 2. YOLO TensorRT 엔진 모델 로드
    print(f"'{ENGINE_MODEL_PATH}' 모델을 로드합니다.")
    try:
        model = YOLO(ENGINE_MODEL_PATH, task='detect')
    except Exception as e:
        print(f"모델 로드 중 오류 발생: {e}")
        return

    # 3. USB 카메라 실행 (V4L2 모드)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("USB 카메라를 열 수 없습니다. 장치 인덱스(/dev/video*) 확인 필요")
        return

    print("USB 카메라가 준비되었습니다. 실시간 탐지를 시작합니다. (종료하려면 'q'를 누르세요)")

    # 4. 실시간 탐지 루프
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # YOLO 추론
        results = model(frame, verbose=False)

        # --- 💡 알림 기능 로직 ---
        target_detected = False
        # 탐지된 모든 객체를 확인
        for r in results:
            for c in r.boxes.cls:
                # 클래스 ID를 실제 이름으로 변환
                class_name = model.names[int(c)]
                # 목표 클래스와 이름이 일치하는지 확인
                if class_name == TARGET_CLASS:
                    target_detected = True
                    break
            if target_detected:
                break

        # 목표 클래스가 감지되었고, 알림음 파일이 로드되었다면
        if target_detected and wave_obj:
            # 현재 다른 소리가 재생 중이 아닐 때만 새로 재생
            if play_obj is None or not play_obj.is_playing():
                print(f"🚨 '{TARGET_CLASS}' 감지! 알림음을 재생합니다.")
                play_obj = wave_obj.play()
        # --- 알림 기능 로직 끝 ---

        # 바운딩 박스 시각화
        annotated_frame = results[0].plot()

        # 화면 출력
        cv2.imshow("YOLO Detection (USB Webcam V4L2)", annotated_frame)

        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 5. 자원 해제
    print("프로그램을 종료합니다.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
