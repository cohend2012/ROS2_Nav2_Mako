#!/usr/bin/env python3
"""Probe: how well do COCO detectors work on the photoreal splat camera?

Renders the robot camera (same intrinsics/mount as splat_camera_node.py) at each tour stop
looking in 4 directions, runs a torchvision COCO detector, and writes an annotated contact
sheet + per-class counts. Used to pick the model and confidence threshold before wiring the
live detector node. Run in m20_splat with --gpus all (needs torchvision).
  python3 detect_probe.py --model frcnn_v2|frcnn_mobile|retina --thr 0.5 --out /out/probe.png
"""
import argparse, math, os, sys, time
import numpy as np
import torch
import yaml
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import splat_camera_node as scn


def load_model(name, dev):
    import torchvision.models.detection as D
    if name == "frcnn_v2":
        w = D.FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
        m = D.fasterrcnn_resnet50_fpn_v2(weights=w)
    elif name == "frcnn_mobile":
        w = D.FasterRCNN_MobileNet_V3_Large_FPN_Weights.DEFAULT
        m = D.fasterrcnn_mobilenet_v3_large_fpn(weights=w)
    else:
        w = D.RetinaNet_ResNet50_FPN_V2_Weights.DEFAULT
        m = D.retinanet_resnet50_fpn_v2(weights=w)
    return m.eval().to(dev), w.meta["categories"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="frcnn_v2")
    ap.add_argument("--thr", type=float, default=0.5)
    ap.add_argument("--out", default="/out/probe.png")
    a = ap.parse_args()
    dev = torch.device("cuda")
    cam = scn.SplatCamera()
    model, cats = load_model(a.model, dev)
    tour = yaml.safe_load(open(scn.SCENE))["tour"]
    tiles, counts, times = [], {}, []
    for (x, y) in tour[:-1]:
        for yaw_deg in (0, 90, 180, 270):
            c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
            rgb, _ = cam.render(np.array([x, y, 0.46]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]))
            t = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255).to(dev)
            t0 = time.time()
            with torch.no_grad():
                out = model([t])[0]
            torch.cuda.synchronize(); times.append(time.time() - t0)
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for box, lab, sc in zip(out["boxes"].cpu().numpy(), out["labels"].cpu().numpy(), out["scores"].cpu().numpy()):
                if sc < a.thr:
                    continue
                name = cats[lab]
                counts[name] = counts.get(name, 0) + 1
                x0, y0, x1, y1 = box.astype(int)
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 255), 2)
                cv2.putText(img, f"{name} {sc:.2f}", (x0, max(14, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(img, f"({x:.1f},{y:.1f}) yaw {yaw_deg}", (6, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(cv2.resize(img, (320, 240)))
    cols = 8
    rows = [np.concatenate(tiles[i:i + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[i:i + cols])), 1)
            for i in range(0, len(tiles), cols)]
    cv2.imwrite(a.out, np.concatenate(rows, 0))
    print(f"[probe] {a.model} thr {a.thr}: {len(tiles)} views, median {1000 * np.median(times[1:]):.0f} ms/frame")
    print("[probe] detections by class:", dict(sorted(counts.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
