"""Shared detection drawing: ONE style for the robot camera panel (detector_node.py) and the
boxes Viser draws in the main 3D view (splat_viewer.py), so they look identical:
class-coloured rectangle + filled tag with "class score distance"."""
import colorsys, zlib
import cv2


def class_colour(name):
    """Stable per-class RGB colour (crc32 hue; str hash() is salted per process)."""
    r, g, b = colorsys.hsv_to_rgb((zlib.crc32(name.encode()) % 360) / 360.0, 0.85, 1.0)
    return int(r * 255), int(g * 255), int(b * 255)


def label_text(cls, score, dist=None):
    return f"{cls} {score:.2f}" + (f" {dist:.1f}m" if dist is not None else "")


def draw_box(img, box, text, colour, scale=1.0):
    """img: RGB uint8 (modified in place); box: (x0, y0, x1, y1) pixels; scale grows line and
    text for larger images (1.0 = the 640x480 robot camera)."""
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    th_line = max(1, int(round(2 * scale)))
    fs = 0.5 * scale
    cv2.rectangle(img, (x0, y0), (x1, y1), colour, th_line)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, max(1, int(round(scale))))
    pad = int(round(3 * scale))
    ty1 = max(th + 2 * pad, y0)
    cv2.rectangle(img, (x0, ty1 - th - 2 * pad), (x0 + tw + 2 * pad, ty1), colour, -1)
    cv2.putText(img, text, (x0 + pad, ty1 - pad), cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 0, 0),
                max(1, int(round(scale))), cv2.LINE_AA)
