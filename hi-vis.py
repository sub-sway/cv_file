import cv2
from ultralytics import YOLO
import simpleaudio as sa
import time

ENGINE_MODEL_PATH = 'HiVisModel.engine'
WAV_PATH = 'Stranger_cut.wav'
PERSON_CLASS = 'person'
HIVIS_CLASS = 'hi-vis'

# 겹침 판정 함수
def compute_ioa(inner, outer):
    """inner(hi-vis), outer(person) 박스 기준 IoA (hi-vis 안에서 겹치는 비율)"""
    x1, y1, x2, y2 = inner
    X1, Y1, X2, Y2 = outer
    inter_x1, inter_y1 = max(x1, X1), max(y1, Y1)
    inter_x2, inter_y2 = min(x2, X2), min(y2, Y2)
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    inner_area = (x2 - x1) * (y2 - y1)
    return inter_area / inner_area if inner_area > 0 else 0.0

def main():
    # --- 알림음 로드 ---
    try:
        wave_obj = sa.WaveObject.from_wave_file(WAV_PATH)
        play_obj = None
    except FileNotFoundError:
        print(f"⚠ 알림음 파일 '{WAV_PATH}' 없음")
        wave_obj, play_obj = None, None

    # --- 모델 로드 ---
    print(f"모델 '{ENGINE_MODEL_PATH}' 로드 중...")
    try:
        model = YOLO(ENGINE_MODEL_PATH, task='detect')
    except Exception as e:
        print(f"모델 로드 실패: {e}")
        return

    # --- 카메라 오픈 ---
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("❌ USB 카메라 열기 실패")
        return

    last_beep_time = 0
    beep_interval = 1.0  # 최소 3초 간격
    safe_threshold = 0.4
    warn_threshold = 0.2

    print("▶ 실시간 탐지 시작 (q 종료)")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠ 프레임 읽기 실패")
            break

        results = model(frame, verbose=False)
        person_boxes, hivis_boxes = [], []

        # 탐지 결과 분류
        for r in results:
            for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                cname = model.names[int(cls)]
                x1, y1, x2, y2 = map(int, box.tolist())
                if cname == PERSON_CLASS:
                    person_boxes.append([x1, y1, x2, y2])
                elif cname in [HIVIS_CLASS, "hivis"]:
                    hivis_boxes.append([x1, y1, x2, y2])

        # 알람 조건 체크
        alarm_triggered = False
        for p in person_boxes:
            # 각 person에 대해 hi-vis 겹침 비율 계산
            ratios = [compute_ioa(h, p) for h in hivis_boxes]
            max_ratio = max(ratios) if ratios else 0.0

            if max_ratio >= safe_threshold:
                status = "✅ 안전 (hi-vis 착용)"
            elif max_ratio >= warn_threshold:
                status = "⚠ 애매함 (부분만 감지됨)"
            else:
                status = "🚨 미착용 (알람)"
                alarm_triggered = True

            print(f"Person 박스 {p} → hi-vis 겹침: {max_ratio:.2f} → {status}")

        # 실제 알람 실행
        if alarm_triggered and wave_obj:
            now = time.time()
            if (now - last_beep_time) > beep_interval:
                print("🚨 외부인(HiVis 미착용) 감지! 알림음 재생")
                play_obj = wave_obj.play()
                last_beep_time = now

        # 시각화
        annotated = results[0].plot()
        cv2.imshow("HiVis Monitoring", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("▶ 프로그램 종료")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
