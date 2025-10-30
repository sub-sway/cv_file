from ultralytics import YOLO

model = YOLO('Crack4.pt')

model.export(
    format='engine',
    imgsz=640,
    half=True,
    workspace=4,
    device=0
)

print("complete")