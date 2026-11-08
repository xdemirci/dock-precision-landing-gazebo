#!/usr/bin/env python3
"""Donanimsiz kendi-kendine test: python3 selftest.py   (kamera / MAVLink / ekran gerekmez)"""
import math

import cv2
import numpy as np

import pose
from config import parse_args
from controller import Controller
from detector import MarkerDetector
from estimator import TargetFilter
from hud import Hud
from mission import LandingSequencer
from vehicle import LANDED_IN_AIR, LANDED_ON_GROUND

cfg = parse_args([])
fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_bn(yaw, pitch, roll):
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rotz(yaw) @ Ry @ Rx


# 1) KF: sabit hedef, gurultulu olcum
rng = np.random.default_rng(0)
kf, t = TargetFilter(), 0.0
for _ in range(60):
    t += 0.05
    kf.predict(t)
    kf.update(t, 3.0 + rng.normal(0, 0.1), -2.0 + rng.normal(0, 0.1), 0.1)
tn, te = kf.pos()
check("KF yakinsama", abs(tn - 3) < 0.07 and abs(te + 2) < 0.07, f"({tn:.3f},{te:.3f})")
before = kf.rejected
kf.predict(t + 0.05)
kf.update(t + 0.05, 9.0, 9.0, 0.1)
check("KF aykiri olcum reddi", kf.rejected == before + 1)

# 2) Zincir: piksel -> aci -> yatay cerceve -> NED (yaw/roll/pitch ile)
cam = pose.CamModel(1920, 1080, 1.151917)
yaw, pitch, roll, h = 0.6, -0.04, 0.05, 3.0
rel_ned = np.array([1.2, -0.7, h])                      # arac -> marker (NED)
pb = rot_bn(yaw, pitch, roll).T @ rel_ned               # govde FRD
pc = np.array([pb[1], -pb[0], pb[2]])                   # kamera (x sag, y asagi, z ileri)
px = (cam.K @ (pc / pc[2]))[:2]
ax, ay = cam.angles(tuple(px))
fwd, right = pose.los_to_level_xy(ax, ay, h, roll, pitch)
n = fwd * math.cos(yaw) - right * math.sin(yaw)
e = fwd * math.sin(yaw) + right * math.cos(yaw)
check("aci->NED zinciri", abs(n - 1.2) < 1e-6 and abs(e + 0.7) < 1e-6, f"({n:.4f},{e:.4f})")
f2, r2, d2 = pose.pnp_to_level(pc, roll, pitch)
check("PnP->yatay cerceve (yukseklik dahil)", abs(d2 - h) < 1e-6 and abs(f2 - fwd * h / h) < 1e-6)

# 3) Detector + PnP: sentetik render
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
N = 400
mimg = cv2.aruco.generateImageMarker(dic, 0, N) if hasattr(cv2.aruco, "generateImageMarker") \
    else cv2.aruco.drawMarker(dic, 0, N)
M = int(N * 1.5)
canvas = np.full((M, M), 255, np.uint8)
o = (M - N) // 2
canvas[o:o + N, o:o + N] = mimg
half = cfg.marker * 1.5 / 2
plane = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)


def render(t_cam, yaw_m):
    R = np.diag([1.0, -1, -1]) @ rotz(yaw_m)
    rvec, _ = cv2.Rodrigues(R)
    px, _ = cv2.projectPoints(plane, rvec, np.array(t_cam, float), cam.K, cam.D)
    Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [M, 0], [M, M], [0, M]]), px.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(canvas, Hm, (cam.w, cam.h), borderValue=127)


det_ok = True
for t_cam, yaw_m in [((0.3, -0.2, 3.0), 0.0), ((-0.8, 0.5, 6.0), 0.5), ((0.1, 0.1, 1.2), -0.9)]:
    img = render(t_cam, yaw_m)
    detector = MarkerDetector(cfg)
    d = None
    for _ in range(cfg.min_hits):
        d = detector.process(img)
    if d is None:
        det_ok = False
        print("   tespit yok", t_cam)
        continue
    pose.solve_pnp(d, cam, cfg.marker)
    d.good = pose.is_good(d, cfg)
    err = np.linalg.norm(d.tvec - np.array(t_cam)) / t_cam[2]
    yaw_img = math.atan2(d.corners[1][1] - d.corners[0][1], d.corners[1][0] - d.corners[0][0])
    ok = d.locked and d.good and err < 0.03 and abs(pose.wrap(d.yaw - yaw_img)) < math.radians(4)
    det_ok &= ok
    print(f"   t={t_cam} -> tvec={np.round(d.tvec, 3)} goreli hata={err * 100:.2f}% reproj={d.reproj:.2f}px "
          f"yaw(pnp)={math.degrees(d.yaw):+.1f} yaw(goruntu)={math.degrees(yaw_img):+.1f}")
check("ArUco tespit + PnP (3 poz)", det_ok)
check("bos goruntude tespit yok", MarkerDetector(cfg).process(np.full((1080, 1920), 127, np.uint8)) is None)

# 4) Inis sirasi: alcal -> son inis -> temas -> disarm dogrula
c = parse_args([])
sq, now = LandingSequencer(c), 0.0
phases, disarm_sent = set(), False
for hh in np.linspace(3.0, 0.1, 30):
    now += 0.2
    out = sq.update(now, True, True, LANDED_IN_AIR, hh, 0.3, 0.05, True)
    phases.add(sq.phase)
now += 0.2
out = sq.update(now, True, True, LANDED_ON_GROUND, 0.05, 0.0, 0.05, True)
disarm_sent |= out["disarm"] == "normal"
now += 0.2
sq.update(now, True, False, LANDED_ON_GROUND, 0.05, 0.0, 0.05, True)
check("inis sirasi", {"DESCEND", "FINAL"} <= phases and disarm_sent and sq.phase == "DONE", f"{sorted(phases)} -> {sq.phase}")
sq2 = LandingSequencer(c)
sq2.update(0.0, True, True, LANDED_ON_GROUND, 0.05, 0.0, 0.05, True)
check("havalanmadan temas/disarm tetiklenmez", sq2.phase != "TOUCH")

# 5) Kontrolcu: hiz ve ivme siniri
ct = Controller(c)
v1 = ct.step(0.05, "ALIGN", True, 10.0, 0.0, 3.0, None, False)
check("ivme siniri", abs(v1[0]) <= c.amax * 0.05 + 1e-9, f"vn={v1[0]:.3f}")
for _ in range(200):
    v = ct.step(0.05, "ALIGN", True, 10.0, 10.0, 3.0, None, False)
check("hiz siniri", math.hypot(v[0], v[1]) <= c.vmax + 1e-6, f"|v|={math.hypot(v[0], v[1]):.3f}")
check("hedef yokken dur", ct.step(0.05, "ALIGN", False, 0, 0, 3.0, None, False)[:3] == (0.0, 0.0, 0.0))

# 5b) Yukseklik: PnP'den ogren, marker kaybolunca EKF z ile devam
from height import MarkerHeight


class _V:
    d, roll, pitch, rng = -8.0, 0.0, 0.0, None

    def rng_ok(self, now):
        return False


class _D:
    good = True
    tvec = np.array([0.2, -0.1, 3.0])


mh, veh_ = MarkerHeight(c), _V()
h1, s1 = mh.update(veh_, _D(), 0.0)                 # arac z=-8 m (yukari), marker 3 m asagida -> m_alt=5
veh_.d = -6.5                                       # 1.5 m alcal, marker artik gorunmuyor
h2, s2 = mh.update(veh_, None, 0.1)
check("yukseklik: PnP ogren + EKF ile devam", abs(h1 - 3.0) < 1e-9 and abs(h2 - 1.5) < 1e-9 and s2 == "ekf*",
      f"{h1:.2f} ({s1}) -> {h2:.2f} ({s2})")

# 5c) h-scale: PnP z 2.5x fazla ise 0.4 ile duzelir
c2 = parse_args(["--h-scale", "0.4"])


class _D2:
    good = True
    tvec = np.array([0.0, 0.0, 7.5])


veh2 = _V()
veh2.d = -4.4
h3, _ = MarkerHeight(c2).update(veh2, _D2(), 0.0)
check("h-scale", abs(h3 - 3.0) < 1e-9, f"{h3:.2f}")

# 6) HUD derleniyor mu (ekransiz)
hud = Hud(c)
info = dict(det=d, tracking=True, running=True, guided=True, mode_name="GUIDED", armed=True, fps=20, h=2.5,
            h_src="ekf", rx=0.1, ry=-0.05, err=0.11, phase_tr="HIZALA", seq_msg="x", seq_failed=False,
            seq_done=False, cmd=(0.1, 0.0, 0.0), yaw_align=True, yaw_err=0.1, alt=5.9, m_alt=-4.4, rng=None, ax_deg=1.0, ay_deg=-2.0)
out_img = hud.render(np.zeros((540, 1080, 3), np.uint8), info)
check("HUD cizimi", out_img.shape == (hud.panel_h + 540, 1080, 3), str(out_img.shape))

print("\nSONUC:", "HEPSI GECTI" if not fails else f"BASARISIZ: {fails}")
raise SystemExit(1 if fails else 0)
