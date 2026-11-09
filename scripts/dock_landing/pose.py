"""Geometri: kamera modeli, PnP, goruntu acisi -> govde/yatay cerceve donusumleri.

Kamera (OpenCV): x sag, y asagi, z ileri.  Kamera asagi bakar, goruntu ustu = govde onu.
Govde FRD: x ileri, y sag, z asagi  ->  p_body = [-cam_y, cam_x, cam_z]
"""
import math

import cv2
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class CamModel:
    """Kalibrasyon yoksa SDF gibi ideal (distorsiyonsuz) kamera."""

    def __init__(self, w, h, hfov, zoom=1.0, calib=None):
        if calib:
            d = np.load(calib)
            self.K = np.asarray(d["camera_matrix" if "camera_matrix" in d else "K"], float)
            self.D = np.asarray(d["distortion_coeffs" if "distortion_coeffs" in d else "D"], float).ravel()
        else:
            fx = (w / 2) / math.tan(hfov / 2) * zoom
            self.K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], float)
            self.D = np.zeros(5)
        self.w, self.h = w, h

    @property
    def cx(self):
        return float(self.K[0, 2])

    @property
    def cy(self):
        return float(self.K[1, 2])

    def angles(self, pt):
        """Piksel -> (angle_x, angle_y) rad. angle_x: sag(+), angle_y: goruntu asagi(+)."""
        u = cv2.undistortPoints(np.array([[pt]], np.float64), self.K, self.D)
        return math.atan(float(u[0, 0, 0])), math.atan(float(u[0, 0, 1]))


def solve_pnp(det, cam, marker_m):
    """Detection'i doldurur: pnp_ok, tvec, reproj, tilt, yaw."""
    L = marker_m / 2
    obj = np.array([[-L, L, 0], [L, L, 0], [L, -L, 0], [-L, -L, 0]], np.float32)   # TL TR BR BL
    pts = det.corners.astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, pts, cam.K, cam.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    det.pnp_ok = bool(ok) and float(tvec.ravel()[2]) > 0
    if not det.pnp_ok:
        return
    proj, _ = cv2.projectPoints(obj, rvec, tvec, cam.K, cam.D)
    det.reproj = float(np.mean(np.linalg.norm(proj.reshape(4, 2) - pts, axis=1)))
    R, _ = cv2.Rodrigues(rvec)
    det.tilt = math.degrees(math.acos(min(1.0, abs(float(R[2, 2])))))
    det.yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    det.tvec = tvec.ravel()


def is_good(det, cfg):
    return det.pnp_ok and det.reproj <= cfg.max_reproj and det.tilt <= cfg.max_tilt


def level_rot(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx                                  # govde -> yatay (yaw'siz) cerceve


def los_to_level_xy(ax, ay, h, roll, pitch):
    """Gorus acisi + yukseklik -> (ileri, sag) [m], roll/pitch duzeltmeli. h: marker duzlemine yukseklik."""
    v = level_rot(roll, pitch) @ np.array([-math.tan(ay), math.tan(ax), 1.0])
    if v[2] <= 0.05:
        return None
    return h * v[0] / v[2], h * v[1] / v[2]


def pnp_to_level(tvec, roll, pitch):
    """PnP tvec (kamera cercevesi) -> (ileri, sag, asagi) [m] yatay cerceve. asagi = marker'a yukseklik."""
    p = level_rot(roll, pitch) @ np.array([-tvec[1], tvec[0], tvec[2]])
    return float(p[0]), float(p[1]), float(p[2])


class YawFilter:
    """Marker yaw'i icin dairesel alcak-gecirgen."""

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, ang):
        self.value = ang if self.value is None else wrap(self.value + self.alpha * wrap(ang - self.value))

    def reset(self):
        self.value = None
