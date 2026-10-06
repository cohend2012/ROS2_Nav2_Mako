#!/usr/bin/env python3
"""Office-object detection on the robot's photoreal camera, in 2D and 3D.

Input (from splat_camera_node.py, same header stamp on both): /camera/image_raw (rgb8) and
/camera/depth (32FC1, z-depth m); robot pose from /odom_true (sim world = Nav2 map frame).

Per frame (~DET_HZ): OWLv2 open-vocabulary detector queried with an office vocabulary
(DET_MODEL=owl, default; DET_MODEL=coco = torchvision FasterRCNN-v2, COCO office subset)
-> filter (score >= DET_THR / per-class, reject frame-filling boxes = splat blur) -> 2D boxes. 3D: median depth in the box's
central region, back-projected through the camera mount into the world. Repeat sightings of
the same class within MERGE_DIST are fused into an OBJECT MAP; an object is "confirmed"
after CONFIRM_N sightings, so one-off hallucinations never reach the map.

Output: /detections/image (rgb8, annotated) and /detections/objects (std_msgs/String JSON:
{"frame": [{cls, score, box, xyz|null}], "objects": [{id, cls, xyz, n, score}]}  # confirmed only)
"""
import json, math, os, sys, threading, time
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from splat_camera_node import W, H, HFOV, CAM_FWD, CAM_UP, R_BASE_OPT, quat_to_mat

DET_MODEL = os.environ.get("DET_MODEL", "owl")             # owl = OWLv2 office vocabulary | coco
DET_HZ = float(os.environ.get("DET_HZ", 3.0 if DET_MODEL == "owl" else 5.0))
DET_THR = float(os.environ.get("DET_THR", 0.3 if DET_MODEL == "owl" else 0.6))
OWL_MODEL = os.environ.get("OWL_MODEL", "google/owlv2-base-patch16-ensemble")
# OWLv2 open-vocabulary queries: office categories drawn from Objects365 / LVIS (COCO lacks
# desk, whiteboard, filing cabinet, printer, trash can, cardboard box, ...). Probe on the
# photoreal camera (tools/splat/detect_probe_owl.py, 2026-10-05): thr 0.3 keeps the real hits
# (ladder 0.80, cones 0.90, whiteboard 0.79, boxes 0.73, doors, chairs, desks) and drops clutter.
OFFICE = ["office chair", "desk", "computer monitor", "laptop", "keyboard", "computer mouse",
          "whiteboard", "filing cabinet", "bookshelf", "printer", "trash can", "cardboard box",
          "coffee mug", "water bottle", "backpack", "potted plant", "door", "telephone",
          "desk lamp", "sofa", "person", "table", "storage cabinet", "microwave",
          "refrigerator", "traffic cone", "ladder", "projector screen", "clock", "book"]
CLASS_THR = {"projector screen": 0.4}                        # fires on dark windows below this
MAX_AREA = float(os.environ.get("DET_MAX_AREA", 0.35))     # boxes covering more of the frame = blur
MAX_RANGE = float(os.environ.get("DET_MAX_RANGE", 8.0))     # don't place objects farther than this
MERGE_DIST = float(os.environ.get("DET_MERGE_DIST", 1.0))
CONFIRM_N = int(os.environ.get("DET_CONFIRM_N", 3))
# DET_MODEL=coco fallback: the COCO classes that are office items (vehicles/animals/"bed" etc.
# were splat-blur false positives)
KEEP = {"person", "backpack", "handbag", "suitcase", "bottle", "cup", "chair", "couch",
        "potted plant", "dining table", "tv", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "refrigerator", "book", "clock", "scissors"}
FX = (W / 2) / math.tan(HFOV / 2)


from det_draw import class_colour, draw_box, label_text   # one drawing style with splat_viewer.py


def make_detector(dev):
    """Returns detect(rgb uint8 HxWx3) -> [(name, score, (x0, y0, x1, y1))] and a model tag."""
    if DET_MODEL == "owl":
        from transformers import Owlv2Processor, Owlv2ForObjectDetection
        proc = Owlv2Processor.from_pretrained(OWL_MODEL)
        model = Owlv2ForObjectDetection.from_pretrained(OWL_MODEL).eval().to(dev)
        text = proc(text=[OFFICE], return_tensors="pt").to(dev)

        def detect(rgb):
            inp = proc(images=rgb, return_tensors="pt").to(dev)
            with torch.no_grad():
                out = model(**inp, input_ids=text["input_ids"], attention_mask=text["attention_mask"])
            side = max(rgb.shape[:2])          # OWLv2 pads to a square: boxes are in that square
            r = proc.post_process_object_detection(out, threshold=DET_THR,
                                                   target_sizes=torch.tensor([[side, side]], device=dev))[0]
            return [(OFFICE[l], float(s), tuple(b)) for b, l, s in
                    zip(r["boxes"].cpu().numpy(), r["labels"].cpu().numpy(), r["scores"].cpu().numpy())]
        return detect, f"OWLv2 office vocabulary ({len(OFFICE)} classes)"

    from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights as Wts
    model = fasterrcnn_resnet50_fpn_v2(weights=Wts.DEFAULT).eval().to(dev)
    cats = Wts.DEFAULT.meta["categories"]

    def detect(rgb):
        with torch.no_grad():
            o = model([torch.from_numpy(rgb.copy()).permute(2, 0, 1).float().div(255).to(dev)])[0]
        return [(cats[l], float(s), tuple(b)) for b, l, s in
                zip(o["boxes"].cpu().numpy(), o["labels"].cpu().numpy(), o["scores"].cpu().numpy())
                if cats[l] in KEEP]
    return detect, "FasterRCNN-v2 COCO (office subset)"


def box3d(depth, box, z_med, pose, depth_band=0.6, stride=3):
    """3D axis-aligned box (world) for a 2D detection: the box's pixels whose depth is within
    depth_band of the object's median depth (drops background/foreground bleed), back-projected
    through the camera mount; 10th-90th percentile extents (robust to stray pixels).
    Returns (center xyz, size xyz) or None."""
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
    d = depth[y0:y1:stride, x0:x1:stride]
    vv, uu = np.mgrid[y0:y1:stride, x0:x1:stride]
    m = (d > 0.05) & (np.abs(d - z_med) < depth_band)
    if m.sum() < 12:
        return None
    z, u, v = d[m], uu[m] + 0.5, vv[m] + 0.5
    p_opt = np.stack([(u - W / 2) / FX * z, (v - H / 2) / FX * z, z], axis=1)
    p_base = p_opt @ R_BASE_OPT.T + np.array([CAM_FWD, 0.0, CAM_UP])
    pw = p_base @ pose[2].T + pose[1]
    lo, hi = np.percentile(pw, 10, axis=0), np.percentile(pw, 90, axis=0)
    lo[2] = max(lo[2], 0.0)                     # never below the floor
    size = np.clip(hi - lo, 0.10, 3.0)
    return ((lo + hi) / 2).tolist(), size.tolist()


class ObjectMap:
    def __init__(self):
        self.objs, self.next_id = [], 1

    def add(self, cls, xyz, score, size=None):
        best, bd = None, MERGE_DIST
        for o in self.objs:
            if o["cls"] == cls:
                d = math.dist(o["xyz"], xyz)
                if d < bd:
                    best, bd = o, d
        if best is None:
            self.objs.append({"id": self.next_id, "cls": cls, "xyz": list(xyz), "n": 1, "score": score,
                              "size": None if size is None else list(size)})
            self.next_id += 1
        else:
            a = 1.0 / min(best["n"] + 1, 10)          # running mean, settles after ~10 sightings
            best["xyz"] = [(1 - a) * p + a * q for p, q in zip(best["xyz"], xyz)]
            if size is not None:
                best["size"] = list(size) if best.get("size") is None else \
                    [(1 - a) * p + a * q for p, q in zip(best["size"], size)]
            best["n"] += 1
            best["score"] = max(best["score"], score)

    def confirmed(self):
        return [dict(o, xyz=[round(v, 2) for v in o["xyz"]], score=round(o["score"], 2),
                     size=None if o.get("size") is None else [round(v, 2) for v in o["size"]])
                for o in self.objs if o["n"] >= CONFIRM_N]


def main():
    import rclpy, cv2
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    dev = torch.device("cuda")
    detect, model_tag = make_detector(dev)

    rclpy.init()
    node = rclpy.create_node("m20_object_detector")
    lock = threading.Lock()
    frames = {}                                   # stamp -> {"rgb": .., "depth": ..}
    poses = []                                    # (t, p, R) recent ground-truth poses
    omap = ObjectMap()

    def key(h):
        return (h.stamp.sec, h.stamp.nanosec)

    def on_rgb(m):
        with lock:
            frames.setdefault(key(m.header), {})["rgb"] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
            frames.setdefault(key(m.header), {})["t"] = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            while len(frames) > 6:
                frames.pop(next(iter(frames)))

    def on_depth(m):
        with lock:
            frames.setdefault(key(m.header), {})["depth"] = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)

    def on_odom(m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        with lock:
            poses.append((t, np.array([p.x, p.y, p.z]), quat_to_mat(o.x, o.y, o.z, o.w)))
            del poses[:-400]

    node.create_subscription(Image, "/camera/image_raw", on_rgb, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth", on_depth, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom_true", on_odom, qos_profile_sensor_data)
    pub_img = node.create_publisher(Image, "/detections/image", qos_profile_sensor_data)
    pub_obj = node.create_publisher(String, "/detections/objects", 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.get_logger().info(f"object detector up: {model_tag}, thr {DET_THR}, {DET_HZ} Hz, "
                           f"confirm after {CONFIRM_N} sightings")

    last_k, last_log, n_frames, t_inf = None, time.time(), 0, []
    while rclpy.ok():
        t_loop = time.time()
        with lock:
            ready = [(k, f) for k, f in frames.items() if "rgb" in f and "depth" in f]
            k, f = ready[-1] if ready else (None, None)
            pose = None
            if f is not None and poses:
                pose = min(poses, key=lambda p: abs(p[0] - f["t"]))
        if f is None or k == last_k:
            time.sleep(0.02)
            continue
        last_k = k
        rgb, depth = f["rgb"], f["depth"]
        t0 = time.time()
        raw = detect(rgb)
        t_inf.append(time.time() - t0)
        det = []
        for name, sc, box in raw:
            x0, y0, x1, y1 = box
            if sc < max(DET_THR, CLASS_THR.get(name, 0.0)) or (x1 - x0) * (y1 - y0) > MAX_AREA * W * H:
                continue
            # 3D: median depth of the box's central 40 %, back-projected through the camera mount
            cx0, cx1 = int(x0 + 0.3 * (x1 - x0)), int(x1 - 0.3 * (x1 - x0)) + 1
            cy0, cy1 = int(y0 + 0.3 * (y1 - y0)), int(y1 - 0.3 * (y1 - y0)) + 1
            patch = depth[cy0:cy1, cx0:cx1]
            patch = patch[patch > 0.05]
            xyz, size = None, None
            if patch.size and pose is not None:
                z = float(np.median(patch))
                if z < MAX_RANGE:
                    b3 = box3d(depth, box, z, pose)
                    if b3 is not None:                  # 3D box: centre + extent from the depth pixels
                        xyz, size = b3
                    else:                               # fallback: ray through the box centre
                        u, v = (x0 + x1) / 2, (y0 + y1) / 2
                        p_opt = np.array([(u - W / 2) / FX * z, (v - H / 2) / FX * z, z])
                        p_base = R_BASE_OPT @ p_opt + np.array([CAM_FWD, 0.0, CAM_UP])
                        xyz = (pose[1] + pose[2] @ p_base).tolist()
                    omap.add(name, xyz, float(sc), size)
            det.append({"cls": name, "score": round(float(sc), 2),
                        "box": [int(x0), int(y0), int(x1), int(y1)],
                        "xyz": None if xyz is None else [round(v, 2) for v in xyz],
                        "size": None if size is None else [round(v, 2) for v in size]})
        # annotated image
        img = rgb.copy()
        for d in det:                   # shared style (det_draw.py) = the boxes in Viser's main view
            dist = math.dist(pose[1][:2], d["xyz"][:2]) if (d["xyz"] and pose) else None
            draw_box(img, d["box"], label_text(d["cls"], d["score"], dist), class_colour(d["cls"]))
        im = Image()
        im.header.stamp = node.get_clock().now().to_msg()
        im.header.frame_id = "camera_optical_link"
        im.height, im.width, im.encoding, im.step = H, W, "rgb8", W * 3
        im.data = img.tobytes()
        pub_img.publish(im)
        pub_obj.publish(String(data=json.dumps({"frame": det, "objects": omap.confirmed()})))
        n_frames += 1
        if time.time() - last_log > 15:
            conf = omap.confirmed()
            by = {}
            for o in conf:
                by[o["cls"]] = by.get(o["cls"], 0) + 1
            node.get_logger().info(f"{n_frames} frames, {1e3 * np.median(t_inf[-50:]):.0f} ms/inference; "
                                   f"object map: {len(conf)} confirmed {by}")
            last_log = time.time()
        time.sleep(max(0.0, 1.0 / DET_HZ - (time.time() - t_loop)))


if __name__ == "__main__":
    main()
