#!/usr/bin/env python3
"""Train the YOLO dial detector on sim-labelled frames and export it for the robot.

    MUJOCO_GL=osmesa python3 tools/perception/make_dial_dataset.py --out /tmp/dials --n 1600
    python3 tools/perception/train_dial_yolo.py --data /tmp/dials/data.yaml --out /tmp/yolo
    python3 tools/perception/eval_dial_detector.py          # envelope + false alarms

Needs `pip install ultralytics` (training only; the robot runs the ONNX with onnxruntime).
Fine-tunes YOLO11n (COCO-pretrained) at 320 px, the robot camera's width. On 4 CPU cores:
~29 min for 30 epochs; 2026-11-06 run: precision 0.96, recall 0.97, mAP50 0.993 on the
held-out 15 %. Writes <out>/dial/weights/best.pt and copies the ONNX export to
src/m20_agent/models/dial_yolo.onnx.

For a real site: add labelled photos from the robot's camera (same YOLO txt format, class
0 = dial) to the dataset and fine-tune from best.pt. Sim-only training is a start, not a
substitute: real dials, glare and clutter differ from the sim's.
"""
import argparse
import os
import shutil
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--init", default="yolo11n.pt", help="weights to start from (a previous best.pt to fine-tune)")
    a = ap.parse_args()
    from ultralytics import YOLO
    t = time.time()
    m = YOLO(a.init)
    m.train(data=a.data, imgsz=320, epochs=a.epochs, batch=32, workers=2, device="cpu",
            project=a.out, name="dial", exist_ok=True, patience=8, plots=False, verbose=False,
            mosaic=1.0, degrees=0.0, fliplr=0.5, hsv_v=0.4, hsv_s=0.5, scale=0.4, close_mosaic=3)
    best = os.path.join(a.out, "dial", "weights", "best.pt")
    onnx = YOLO(best).export(format="onnx", imgsz=320, opset=12, simplify=False, dynamic=False)
    dst = os.path.join(REPO, "src", "m20_agent", "models", "dial_yolo.onnx")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy(onnx, dst)
    print(f"trained in {time.time() - t:.0f} s -> {dst}")


if __name__ == "__main__":
    main()
