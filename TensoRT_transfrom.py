from ultralytics import YOLO

MODELPATH = '' # 모델 절대 경로 입력
model = YOLO(MODELPATH)

model.export(
    format='engine',
    imgsz=640,
    half=True,
    workspace=4,
    device=0
)

print("complete")
