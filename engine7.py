import os
import subprocess

# 경로 고정 (export.py 있는 곳)
YOLOV7_DIR = "/home/orin/Desktop/cv/yolov7"
PT_MODEL_PATH = "/home/orin/Desktop/cv/carcrack.pt"
ONNX_MODEL_PATH = "/home/orin/Desktop/cv/carcrack2.onnx"
ENGINE_MODEL_PATH = "/home/orin/Desktop/cv/carcrack2.engine"

def convert_to_onnx():
    print("[1] PyTorch → ONNX 변환 시작")
    cmd = [
        "python", f"{YOLOV7_DIR}/export.py",
        "--weights", PT_MODEL_PATH,
        "--grid", "--end2end", "--simplify",
        "--topk-all", "100",
        "--iou-thres", "0.65",
        "--conf-thres", "0.35",
        "--max-wh", "640",
        "--img-size", "640", "640"
    ]
    subprocess.run(cmd, check=True)
    print("[1] ONNX 변환 완료:", ONNX_MODEL_PATH)

def convert_to_engine():
    print("[2] ONNX → TensorRT 변환 시작")
    cmd = [
        "/usr/src/tensorrt/bin/trtexec",
        f"--onnx={ONNX_MODEL_PATH}",
        f"--saveEngine={ENGINE_MODEL_PATH}",
        "--fp16",
        "--workspace=2048"
    ]
    subprocess.run(cmd, check=True)
    print("[2] TensorRT 엔진 생성 완료:", ENGINE_MODEL_PATH)

if __name__ == "__main__":
    if not os.path.exists(PT_MODEL_PATH):
        raise FileNotFoundError(f"{PT_MODEL_PATH} 파일이 없습니다.")
    convert_to_onnx()
    convert_to_engine()
