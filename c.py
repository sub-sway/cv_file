from ultralytics.utils.torch_utils import select_device
from ultralytics.utils.ops import non_max_suppression
import cv2

# TensorRT engine 직접 로드
from ultralytics.engine.exporter import TRTBackend
backend = TRTBackend("carcrack.engine")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # inference
    im = backend.preprocess(frame)
    preds = backend(im)  # TensorRT inference 결과

    # NMS
    preds = non_max_suppression(preds, 0.25, 0.45)

    # 후처리 및 시각화
    annotated = backend.postprocess(preds, frame)
    cv2.imshow("TRT inference", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
