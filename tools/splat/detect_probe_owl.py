#!/usr/bin/env python3
"""Probe: open-vocabulary OFFICE detection (OWLv2) on the photoreal splat camera.

Same views as detect_probe.py (each tour stop x 4 headings). OWLv2 (google/owlv2-base-
patch16-ensemble, pretrained on web-scale image-text + detection data) is queried with an
office vocabulary drawn from Objects365/LVIS office categories, so it can find classes COCO
lacks (desk, whiteboard, filing cabinet, printer, trash can, cardboard box, ...).
  python3 detect_probe_owl.py --thr 0.25 --out /out/probe_owl.png
"""
import argparse, math, os, sys, time
import numpy as np
import torch
import yaml
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import splat_camera_node as scn

OFFICE = ["office chair", "desk", "computer monitor", "laptop", "keyboard", "computer mouse",
          "whiteboard", "filing cabinet", "bookshelf", "printer", "trash can", "cardboard box",
          "coffee mug", "water bottle", "backpack", "potted plant", "door", "telephone",
          "desk lamp", "sofa", "person", "table", "storage cabinet", "microwave",
          "refrigerator", "traffic cone", "ladder", "projector screen", "clock", "book"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--thr", type=float, default=0.25)
    ap.add_argument("--out", default="/out/probe_owl.png")
    ap.add_argument("--model", default="google/owlv2-base-patch16-ensemble")
    a = ap.parse_args()
    from transformers import Owlv2Processor, Owlv2ForObjectDetection
    dev = torch.device("cuda")
    proc = Owlv2Processor.from_pretrained(a.model)
    model = Owlv2ForObjectDetection.from_pretrained(a.model).eval().to(dev)
    text = proc(text=[OFFICE], return_tensors="pt").to(dev)
    cam = scn.SplatCamera()
    tour = yaml.safe_load(open(scn.SCENE))["tour"]
    tiles, counts, times = [], {}, []
    for (x, y) in tour[:-1]:
        for yaw_deg in (0, 90, 180, 270):
            c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
            rgb, _ = cam.render(np.array([x, y, 0.46]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]))
            t0 = time.time()
            inp = proc(images=rgb, return_tensors="pt").to(dev)
            with torch.no_grad():
                out = model(**inp, input_ids=text["input_ids"], attention_mask=text["attention_mask"])
            # OWLv2 pads to a square: boxes are relative to the padded square (side = max(W, H))
            side = max(rgb.shape[:2])
            res = proc.post_process_object_detection(out, threshold=a.thr,
                                                     target_sizes=torch.tensor([[side, side]], device=dev))[0]
            torch.cuda.synchronize(); times.append(time.time() - t0)
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for box, lab, sc in zip(res["boxes"].cpu().numpy(), res["labels"].cpu().numpy(), res["scores"].cpu().numpy()):
                name = OFFICE[lab]
                x0, y0, x1, y1 = box.astype(int)
                if (x1 - x0) * (y1 - y0) > 0.35 * rgb.shape[0] * rgb.shape[1]:
                    continue
                counts[name] = counts.get(name, 0) + 1
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 255), 2)
                cv2.putText(img, f"{name} {sc:.2f}", (x0, max(14, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            tiles.append(cv2.resize(img, (320, 240)))
    cols = 8
    rows = [np.concatenate(tiles[i:i + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[i:i + cols])), 1)
            for i in range(0, len(tiles), cols)]
    cv2.imwrite(a.out, np.concatenate(rows, 0))
    print(f"[probe] OWLv2 thr {a.thr}: {len(tiles)} views, median {1000 * np.median(times[1:]):.0f} ms/frame")
    print("[probe] detections by class:", dict(sorted(counts.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
