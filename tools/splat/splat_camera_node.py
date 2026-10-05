#!/usr/bin/env python3
"""Photoreal camera for splat worlds: renders the 3D Gaussian splat with gsplat (CUDA)
at the sim's ground-truth robot pose.

MuJoCo simulates physics + LiDAR on boxes extracted from the splat
(tools/splat/extract_geometry.py); this node supplies what a camera would see.
Frames: /odom_true pose is the sim world (= `map`); the splat is moved into that frame
with the scene yaml's splat_to_world transform (the same one the boxes were built with).

Publishes (ROS_DOMAIN_ID from env):
  /camera/image_raw    sensor_msgs/Image rgb8, frame camera_optical_link
  /camera/camera_info  sensor_msgs/CameraInfo (pinhole, no distortion)
  /camera/depth        sensor_msgs/Image 32FC1 metres (SPLAT_DEPTH=1, default on)
  static TF base_link -> camera_optical_link
Env: SPLAT_PLY, SPLAT_SCENE (scene yaml), CAM_W, CAM_H, CAM_HFOV_DEG, CAM_HZ,
     CAM_FWD, CAM_UP (camera offset from base_link, m).

Offline check (no ROS):  python3 splat_camera_node.py --snapshot X Y YAW_DEG OUT_PREFIX
  -> OUT_PREFIX.png (rgb) + OUT_PREFIX_depth.npy, camera at world pose (X, Y, base z 0.45).
Camera mount defaults to the vendor URDF front_camera (CAM_FWD/CAM_UP override).
"""
import math, os, struct, sys, zlib
import numpy as np
import torch
import yaml
from gsplat import rasterization

PLY = os.environ.get("SPLAT_PLY", "/splat/gaussians_indoor.ply")
SCENE = os.environ.get("SPLAT_SCENE", "/splat/indoor_splat.scene.yaml")
W, H = int(os.environ.get("CAM_W", 640)), int(os.environ.get("CAM_H", 480))
HFOV = math.radians(float(os.environ.get("CAM_HFOV_DEG", 90)))
HZ = float(os.environ.get("CAM_HZ", 10))
# default = vendor URDF front_camera_joint (xyz 0.37646 0 0.03738, level, facing forward;
# translation from the hardware doc, rotation ASSUMED per the URDF README)
CAM_FWD, CAM_UP = float(os.environ.get("CAM_FWD", 0.37646)), float(os.environ.get("CAM_UP", 0.03738))
DEPTH = os.environ.get("SPLAT_DEPTH", "1") == "1"
SNAPSHOT_BASE_Z = 0.45   # nominal standing base height for --snapshot

# base_link (x fwd, y left, z up) -> optical (x right, y down, z fwd); columns = optical axes in base
R_BASE_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def load_splat(ply_path, splat_to_world, device):
    with open(ply_path, "rb") as f:
        props, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            elif line.startswith("property"):
                props.append(line.split()[-1])
            elif line == "end_header":
                break
        data = np.fromfile(f, dtype=np.float32, count=n * len(props)).reshape(n, len(props))
    col = {k: i for i, k in enumerate(props)}
    g = lambda *ks: data[:, [col[k] for k in ks]]
    T = np.asarray(splat_to_world, np.float64)
    means = g("x", "y", "z") @ T[:3, :3].T.astype(np.float32) + T[:3, 3].astype(np.float32)
    quats = g("rot_0", "rot_1", "rot_2", "rot_3")                       # wxyz
    if not np.allclose(T[:3, :3], np.eye(3)):
        raise SystemExit("splat_to_world with rotation not supported yet (extractor emits translation only)")
    scales = np.exp(g("scale_0", "scale_1", "scale_2"))
    opac = 1.0 / (1.0 + np.exp(-data[:, col["opacity"]]))
    dc = g("f_dc_0", "f_dc_1", "f_dc_2")[:, None, :]
    n_rest = sum(k.startswith("f_rest_") for k in col)
    if n_rest:  # stored channel-major: [R0..R14, G0..G14, B0..B14]
        rest = g(*[f"f_rest_{i}" for i in range(n_rest)]).reshape(n, 3, n_rest // 3).transpose(0, 2, 1)
        sh = np.concatenate([dc, rest], 1)
    else:
        sh = dc
    deg = int(round(math.sqrt(sh.shape[1]))) - 1
    t = lambda a: torch.from_numpy(np.ascontiguousarray(a, np.float32)).to(device)
    return dict(means=t(means), quats=t(quats), scales=t(scales), opacities=t(opac), colors=t(sh)), deg


class SplatCamera:
    def __init__(self):
        if not torch.cuda.is_available():
            raise SystemExit("CUDA not available — run the container with --gpus all")
        self.dev = torch.device("cuda")
        with open(SCENE) as f:
            self.scene = yaml.safe_load(f)
        self.g, self.deg = load_splat(PLY, self.scene["splat_to_world"], self.dev)
        fx = (W / 2) / math.tan(HFOV / 2)
        self.K = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]], np.float64)
        self.Kt = torch.tensor(self.K, dtype=torch.float32, device=self.dev)[None]
        print(f"[splat_cam] {len(self.g['means'])} Gaussians (SH deg {self.deg}) on {torch.cuda.get_device_name()}, "
              f"{W}x{H} hfov {math.degrees(HFOV):.0f} deg", flush=True)

    def render(self, p_base, R_base):
        """p_base (3,), R_base (3,3) world<-base. Returns rgb uint8 HxWx3, depth float32 HxW."""
        R_wc = R_base @ R_BASE_OPT
        p_c = p_base + R_base @ np.array([CAM_FWD, 0.0, CAM_UP])
        view = np.eye(4)
        view[:3, :3] = R_wc.T
        view[:3, 3] = -R_wc.T @ p_c
        viewt = torch.tensor(view, dtype=torch.float32, device=self.dev)[None]
        with torch.no_grad():
            out, alpha, _ = rasterization(
                self.g["means"], self.g["quats"], self.g["scales"], self.g["opacities"], self.g["colors"],
                viewt, self.Kt, W, H, sh_degree=self.deg, near_plane=0.05, far_plane=200.0,
                render_mode="RGB+ED" if DEPTH else "RGB")
        img = out[0]
        rgb = (img[..., :3].clamp(0, 1) * 255).to(torch.uint8).cpu().numpy()
        depth = None
        if DEPTH:
            a = alpha[0, ..., 0]
            depth = torch.where(a > 0.5, img[..., 3], torch.zeros_like(a)).cpu().numpy().astype(np.float32)
        return rgb, depth


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def snapshot(x, y, yaw_deg, prefix):
    cam = SplatCamera()
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    rgb, depth = cam.render(np.array([x, y, SNAPSHOT_BASE_Z]), R)
    write_png(prefix + ".png", rgb)
    if depth is not None:
        np.save(prefix + "_depth.npy", depth)
    print(f"[splat_cam] wrote {prefix}.png", flush=True)


def ros_main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    class Node_(Node):
        def __init__(self):
            super().__init__("m20_splat_camera")
            self.cam = SplatCamera()
            self.pose = None
            self.create_subscription(Odometry, "/odom_true", self._odom, qos_profile_sensor_data)
            self.pub_img = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
            self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
            self.pub_depth = self.create_publisher(Image, "/camera/depth", qos_profile_sensor_data) if DEPTH else None
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = "base_link", "camera_optical_link"
            tf.transform.translation.x, tf.transform.translation.z = CAM_FWD, CAM_UP
            # optical frame rotation (R_BASE_OPT) as a quaternion: (x,y,z,w) = (-0.5, 0.5, -0.5, 0.5)
            q = tf.transform.rotation
            q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
            self.stf = StaticTransformBroadcaster(self)
            self.stf.sendTransform(tf)
            self.n = 0
            self.create_timer(1.0 / HZ, self._tick)
            self.get_logger().info(f"splat camera up: {W}x{H} @ {HZ} Hz from /odom_true")

        def _odom(self, msg):
            self.pose = msg.pose.pose

        def _tick(self):
            if self.pose is None:
                return
            p, o = self.pose.position, self.pose.orientation
            rgb, depth = self.cam.render(np.array([p.x, p.y, p.z]), quat_to_mat(o.x, o.y, o.z, o.w))
            stamp = self.get_clock().now().to_msg()
            im = Image()
            im.header.stamp, im.header.frame_id = stamp, "camera_optical_link"
            im.height, im.width, im.encoding, im.step = H, W, "rgb8", W * 3
            im.data = rgb.tobytes()
            self.pub_img.publish(im)
            info = CameraInfo()
            info.header = im.header
            info.height, info.width, info.distortion_model = H, W, "plumb_bob"
            info.d = [0.0] * 5
            info.k = self.cam.K.reshape(-1).tolist()
            info.r = np.eye(3).reshape(-1).tolist()                       # rclpy wants floats, not ints
            info.p = np.hstack([self.cam.K, np.zeros((3, 1))]).reshape(-1).tolist()
            self.pub_info.publish(info)
            if self.pub_depth is not None:
                dm = Image()
                dm.header = im.header
                dm.height, dm.width, dm.encoding, dm.step = H, W, "32FC1", W * 4
                dm.data = depth.tobytes()
                self.pub_depth.publish(dm)
            self.n += 1
            if self.n % int(10 * HZ) == 0:
                self.get_logger().info(f"{self.n} frames")

    rclpy.init()
    node = Node_()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    if len(sys.argv) >= 6 and sys.argv[1] == "--snapshot":
        snapshot(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5])
    else:
        ros_main()
