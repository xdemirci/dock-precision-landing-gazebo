#!/usr/bin/env python3
"""
ArUco ile dock ortalama + inis.

Iki calisma sekli:
  A) Varsayilan: kendi PosVel KF'imiz (ArduPilot precland KULLANMADAN), hepsi GUIDED modda:
       HIZALA -> ALCAL -> SON INIS (marker kaybolsa da KF ile) -> TEMAS -> DISARM gonder -> DISARM dogrula
  B) --precland: marker gorulurken LANDING_TARGET gonderilir (ArduPilot AC_PrecLand, PLND_TYPE=1);
       ortalaninca LAND moduna gecilir, inis + disarm ArduPilot'ta (biz sadece dogrulariz)

Secenekler:
  --precland      ArduPilot precland ile in (--land-mode'u da acar)
  --land-mode     son asamada LAND moduna gec (LANDING_TARGET gondermeden)
  --no-descend    sadece ortala, inme  (eski adi: --no-land)
  --no-disarm     yere inince disarm gonderme
  --force-disarm  normal disarm reddedilirse (zaman asimindan sonra, yerdeyken) zorla disarm

Tuslar:  SPACE = start/pause   y = marker'a yonelme ac/kapa   q = cikis

ArduPilot parametreleri (--precland icin, sonra SITL'i yeniden baslat):
    PLND_ENABLED 1, PLND_TYPE 1, PLND_EST_TYPE 1, PLND_LAG 0.15, PLND_STRICT 1
    + rangefinder (RNGFND1_*) acik olmali

    python3 precland_center.py --conn udp:127.0.0.1:14550
    python3 precland_center.py --yaw-align
    python3 precland_center.py --precland --land-h 3
    python3 precland_center.py --vmax 0.3 --disp-w 800
"""
import argparse
import math
import time
from collections import deque

import cv2
import numpy as np
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--conn", default="udp:127.0.0.1:14550")
ap.add_argument("--port", type=int, default=5600)
ap.add_argument("--disp-w", type=int, default=1080, help="pencere genisligi (px)")
ap.add_argument("--marker", type=float, default=0.4165)
ap.add_argument("--hfov", type=float, default=1.151917)
ap.add_argument("--zoom", type=float, default=1.0)
ap.add_argument("--id", type=int, default=-1)
ap.add_argument("--kp", type=float, default=0.5, help="konum hatasi -> hiz (1/s)")
ap.add_argument("--vmax", type=float, default=0.5, help="max yatay hiz m/s")
ap.add_argument("--amax", type=float, default=0.6, help="max yatay ivme m/s^2 (yumusatma)")
ap.add_argument("--tol", type=float, default=0.15, help="ortalandi esigi (m)")
ap.add_argument("--noise", type=float, default=0.02, help="olcum sigma = noise * yukseklik")
ap.add_argument("--yaw-align", action="store_true", help="baslangicta marker'a yonelme acik")
ap.add_argument("--kyaw", type=float, default=0.8, help="yaw hata(rad) -> yaw hizi (rad/s)")
ap.add_argument("--yawmax", type=float, default=30.0, help="max yaw hizi deg/s")
# --- inis / disarm ---
ap.add_argument("--precland", action="store_true",
                help="ArduPilot precland: LANDING_TARGET gonder, ortalaninca LAND'e gec")
ap.add_argument("--land-mode", action="store_true",
                help="son asamada LAND moduna gec (varsayilan: GUIDED ile yere kadar in + disarm)")
ap.add_argument("--no-descend", "--no-land", dest="no_descend", action="store_true",
                help="sadece ortala, inme")
ap.add_argument("--no-disarm", action="store_true", help="yere inince disarm gonderme")
ap.add_argument("--force-disarm", action="store_true",
                help="normal disarm reddedilirse zorla disarm (21196). DIKKAT: motorlari aninda keser")
ap.add_argument("--final-h", type=float, default=1.0,
                help="bu yukseklikten asagida marker kaybolsa da KF ile dik inise devam (m)")
ap.add_argument("--vfinal", type=float, default=0.2, help="min inis hizi m/s")
ap.add_argument("--touch-h", type=float, default=0.6,
                help="yedek temas tespiti: bu yukseklikten asagida ve dikey hiz ~0 (m)")
ap.add_argument("--disarm-timeout", type=float, default=6.0,
                help="normal disarm bu surede kabul edilmezse uyari (ve --force-disarm varsa zorla) (s)")
ap.add_argument("--land-h", type=float, default=1.0, help="--land-mode/--precland: LAND'e gecis yuksekligi (m)")
ap.add_argument("--lost-s", type=float, default=1.0, help="marker kaybi: KF ile devam suresi (s)")
ap.add_argument("--final-lost-s", type=float, default=20.0, help="son inis asamasinda KF ile devam suresi (s)")
args = ap.parse_args()
if args.precland:
    args.land_mode = True


# ---------------- PosVelEKF (AP PosVelEKF.cpp portu) ----------------
class PosVelEKF:
    def __init__(self):
        self.x = np.zeros(2)            # [pos, vel]
        self.P = np.zeros(3)            # [P00, P01, P11]

    def init(self, pos, pos_var, vel, vel_var):
        self.x[:] = (pos, vel)
        self.P[:] = (pos_var, 0.0, vel_var)

    def predict(self, dt, dvel, dvel_noise):
        x, P = self.x, self.P
        self.x = np.array([x[0] + dt * x[1], x[1] + dvel])
        self.P = np.array([P[0] + 2 * dt * P[1] + dt * dt * P[2],
                           P[1] + dt * P[2],
                           P[2] + dvel_noise ** 2])

    def nis(self, pos, pos_var):
        r = pos - self.x[0]
        return r * r / (self.P[0] + pos_var)

    def fuse_pos(self, pos, pos_var):
        x, P = self.x, self.P
        S = P[0] + pos_var
        r = pos - x[0]
        k0, k1 = P[0] / S, P[1] / S
        self.x = np.array([x[0] + k0 * r, x[1] + k1 * r])
        self.P = np.array([P[0] * pos_var / S, P[1] * pos_var / S, P[2] - P[1] * P[1] / S])


class TargetFilter:
    """Sabit hedef: N/E icin 2 adet PosVelEKF. Durum = hedefin yerel NED konumu."""
    NIS_GATE = 9.0
    LOST_REINIT_S = 2.0

    def __init__(self):
        self.f = [PosVelEKF(), PosVelEKF()]
        self.ok = False
        self.t_last = 0.0
        self.t_pred = 0.0
        self.rejected = 0

    def predict(self, now):
        if not self.ok:
            return
        dt = now - self.t_pred
        self.t_pred = now
        for k in self.f:
            k.predict(dt, 0.0, 0.05 * dt)

    def update(self, now, tn, te, sigma):
        var = sigma * sigma
        if not self.ok or now - self.t_last > self.LOST_REINIT_S:
            self.f[0].init(tn, var, 0.0, 0.05)
            self.f[1].init(te, var, 0.0, 0.05)
            self.ok, self.t_pred = True, now
        else:
            if self.f[0].nis(tn, var) + self.f[1].nis(te, var) > 2 * self.NIS_GATE:
                self.rejected += 1
                return False
            self.f[0].fuse_pos(tn, var)
            self.f[1].fuse_pos(te, var)
        self.t_last = now
        return True

    def pos(self):
        return self.f[0].x[0], self.f[1].x[0]


# ---------------- Inis / disarm sirasi (saf mantik, MAVLink'ten bagimsiz) ----------------
LANDED_ON_GROUND = 1      # MAV_LANDED_STATE_ON_GROUND
LANDED_IN_AIR = 2         # MAV_LANDED_STATE_IN_AIR


class LandingSequencer:
    IDLE, ALIGN, DESCEND, FINAL, TOUCH, LAND, DONE = (
        "IDLE", "ALIGN", "DESCEND", "FINAL", "TOUCH", "LAND", "DONE")

    def __init__(self, a):
        self.a = a
        self.reset()

    def reset(self):
        self.phase = self.IDLE
        self.final = False          # son inis kilidi: marker kaybolsa da KF ile devam
        self.airborne = False       # havalanma gorulmeden temas/disarm tetiklenmez
        self.t_still = None
        self.t_touch = None
        self.t_disarm = -1e9
        self.tries = 0
        self.ack = None
        self.failed = False
        self.msg = ""

    def on_ack(self, result):
        self.ack = result

    def _finish(self, msg):
        self.phase = self.DONE
        self.msg = msg
        self.failed = False

    def update(self, now, running, armed, landed, h_raw, vd, err, tracking):
        """Dondurur: {'disarm': None|'normal'|'force', 'land': bool}"""
        a = self.a
        out = {"disarm": None, "land": False}

        if self.phase == self.DONE:
            return out
        if not running:
            if self.phase != self.IDLE:
                self.reset()
            return out

        # havada olduguna dair kanit: landed_state IN_AIR, yukseklik > 0.5 m ya da asagi dogru hareket
        if armed is True and (landed == LANDED_IN_AIR or h_raw > 0.5 or vd > 0.1):
            self.airborne = True

        # ---- temas / LAND sonrasi: disarm gonder ve DOGRULA ----
        if self.phase in (self.TOUCH, self.LAND):
            return self._post_step(now, armed, h_raw, vd, out)

        # ---- ucus fazlari ----
        if self.final and err is not None and err > 3 * a.tol:
            self.final = False                      # hata buyudu: son inisi iptal, tekrar hizala

        if a.land_mode:
            if (not a.no_descend and tracking and err is not None
                    and h_raw < a.land_h and err < a.tol):
                self.phase = self.LAND
                out["land"] = True
                self.msg = "LAND moduna gecildi"
                return out
        elif not a.no_descend and tracking and err is not None and err < 2 * a.tol and h_raw < a.final_h:
            self.final = True

        if tracking and err is not None:
            if a.no_descend or (err >= 2 * a.tol and not self.final):
                self.phase = self.ALIGN
            else:
                self.phase = self.FINAL if self.final else self.DESCEND
        else:
            self.phase = self.FINAL if self.final else self.ALIGN

        # ---- temas tespiti (sadece GUIDED inis akisi) ----
        if (not a.land_mode and not a.no_descend and self.airborne and armed is True
                and self.phase in (self.DESCEND, self.FINAL)):
            still = False
            if self.phase == self.FINAL and h_raw < a.touch_h and abs(vd) < 0.05:
                if self.t_still is None:
                    self.t_still = now
                still = (now - self.t_still) > 1.5
            else:
                self.t_still = None
            if landed == LANDED_ON_GROUND or still:
                self.t_touch = now
                self.t_disarm = -1e9
                if a.no_disarm:
                    self._finish("YERDE (disarm kapali)")
                else:
                    self.phase = self.TOUCH
                    self.msg = "TEMAS: disarm gonderiliyor"
                    return self._post_step(now, armed, h_raw, vd, out)   # ayni adimda gonder
        return out

    def _post_step(self, now, armed, h_raw, vd, out):
        a = self.a
        if armed is False:
            self._finish("DISARM DOGRULANDI (heartbeat: motorlar kapali)")
            return out
        if self.phase == self.LAND:
            self.msg = "LAND modu: ArduPilot'un otomatik disarm'i bekleniyor"
            return out
        elapsed = now - self.t_touch
        if now - self.t_disarm >= 1.0:
            safe_to_force = h_raw < 1.0 and abs(vd) < 0.3
            force = a.force_disarm and elapsed > a.disarm_timeout and safe_to_force
            out["disarm"] = "force" if force else "normal"
            self.t_disarm = now
            self.tries += 1
        if elapsed > a.disarm_timeout and not a.force_disarm:
            self.failed = True
            self.msg = "DISARM KABUL EDILMEDI: manuel disarm gerekli"
        else:
            self.msg = f"DISARM gonderildi (deneme {self.tries}), dogrulama bekleniyor"
        return out


# ---------------- ArUco ----------------
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create")
          else cv2.aruco.DetectorParameters())
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
if hasattr(cv2.aruco, "ArucoDetector"):
    detect = cv2.aruco.ArucoDetector(dic, params).detectMarkers
else:
    detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=params)[:3]

pipeline = (
    f"udpsrc port={args.port} caps=\"application/x-rtp, media=(string)video, "
    "clock-rate=(int)90000, encoding-name=(string)H264\" ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false")
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# ---------------- MAVLink ----------------
m = mavutil.mavlink_connection(args.conn)
m.wait_heartbeat()
print("heartbeat ok")
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
# landed_state (EXTENDED_SYS_STATE) 5 Hz
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 200000, 0, 0, 0, 0, 0)

st = dict(n=0.0, e=0.0, d=0.0, vn=0.0, ve=0.0, vd=0.0, roll=0.0, pitch=0.0, yaw=0.0,
          mode=-1, pos_ok=False, armed=None, landed=0)
MODE_GUIDED = 4
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
              6: "RTL", 9: "LAND", 16: "POSHOLD"}
seq = LandingSequencer(args)


def poll_mav():
    # NOT: m.motors_disarmed_wait() gibi pymavlink yardimcilari bloklar ve video dongusunu dondurur.
    # Bu yuzden armed durumu HEARTBEAT.base_mode'dan, bloklamadan okunur.
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            st.update(n=msg.x, e=msg.y, d=msg.z, vn=msg.vx, ve=msg.vy, vd=msg.vz, pos_ok=True)
        elif t == "ATTITUDE":
            st.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
        elif (t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system
              and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            st["mode"] = msg.custom_mode
            st["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "EXTENDED_SYS_STATE":
            st["landed"] = msg.landed_state
        elif t == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            seq.on_ack(msg.result)


def send_vel_ned(vn, ve, vz=0.0, yaw_rate=None):
    # 0x0FC7: sadece hiz | 0x07C7: hiz + yaw_rate
    mask = 0x0FC7 if yaw_rate is None else 0x07C7
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        0, 0, 0, vn, ve, vz, 0, 0, 0, 0, yaw_rate or 0.0)


def send_disarm(force=False):
    # param1=0: disarm; param2=0: yerdeyken (guvenlik kontrolleriyle), 21196: havada bile zorla
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            0, 21196 if force else 0, 0, 0, 0, 0, 0)


def send_landing_target(ax, ay):
    # ArduPilot AC_PrecLand_MAVLink: vec = (-tan(angle_y), tan(angle_x), 1) BODY_FRD
    # distance=0 -> yukseklik rangefinder'dan alinir. Isaret ters cikarsa: send_landing_target(ax, -ay)
    m.mav.landing_target_send(int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                              ax, ay, 0.0, 0.0, 0.0)


def los_to_level_xy(ax, ay, h):
    """Aci -> (ileri, sag) konum [m]; roll/pitch duzeltmeli, yaw'siz."""
    v = np.array([-math.tan(ay), math.tan(ax), 1.0])
    cr, sr = math.cos(st["roll"]), math.sin(st["roll"])
    cp, sp = math.cos(st["pitch"]), math.sin(st["pitch"])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    vl = Ry @ Rx @ v
    if vl[2] <= 0.05:
        return None
    return h * vl[0] / vl[2], h * vl[1] / vl[2]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------- HUD (kucuk, sade: yesil / beyaz / kirmizi) ----------------
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, WHITE, RED = (0, 255, 0), (255, 255, 255), (60, 60, 255)
fs = max(0.42, args.disp_w / 2300)           # normal satir olcegi (1080 px pencerede ~0.47)
BIG = 1.0                                    # tum satirlar ayni olcek
TH, TH_BIG = 1, 1                            # tum satirlar ayni kalinlik
PAD = int(fs * 14)
ROW_H = int(fs * 50)
BIG_H = int(fs * BIG * 50)
MSG_H = int(fs * 44)
PANEL_H = 2 * PAD + 2 * ROW_H + BIG_H + MSG_H


def draw_row(img, y_top, row_h, items, scale, thick):
    W = img.shape[1] - 2 * PAD

    def measure(sc):
        return [cv2.getTextSize(t, FONT, sc, thick)[0][0] for t, _ in items]

    gap = int(36 * scale)
    total = sum(measure(scale)) + gap * (len(items) - 1)
    if total > W:                             # tasarsa otomatik kucult
        scale *= W / total
        gap = int(36 * scale)
    ws = measure(scale)
    x = PAD
    base = y_top + int(row_h * 0.72)
    for k, (t, col) in enumerate(items):
        cv2.putText(img, t, (x, base), FONT, scale, col, thick, cv2.LINE_AA)
        x += ws[k] + gap // 2
        if k < len(items) - 1:
            cv2.line(img, (x, y_top + int(row_h * 0.2)), (x, y_top + int(row_h * 0.8)), (90, 90, 90), 1)
        x += gap // 2


def build_panel(width, rows):
    panel = np.full((PANEL_H, width, 3), 28, dtype=np.uint8)
    y = PAD
    for k, (items, scale, thick, hgt) in enumerate(rows):
        draw_row(panel, y, hgt, items, scale, thick)
        y += hgt
        if k < len(rows) - 1:
            cv2.line(panel, (PAD, y), (width - PAD, y), (70, 70, 70), 1)
    return panel


PHASE_TR = {"IDLE": "BEKLEMEDE", "ALIGN": "HIZALA", "DESCEND": "ALCAL", "FINAL": "SON INIS",
            "TOUCH": "TEMAS", "LAND": "LAND MODU", "DONE": "BITTI"}

# ---------------- Ana dongu ----------------
kf = TargetFilter()
win = "Precision Landing GUI"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
K = None
w = h_img = 0
s = 1.0
fx = ccx = ccy = 0.0

running = False                 # baslangicta PAUSE
yaw_align = args.yaw_align
cmd_vn = cmd_ve = cmd_vz = 0.0
yaw_f = None                    # filtrelenmis marker yaw (govdeye gore, rad)
arrivals = deque(maxlen=30)
t_prev = None
last_print = 0.0
frames = hits = 0
prev_phase = seq.phase

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        now = time.monotonic()
        dt = (now - t_prev) if t_prev else 0.05
        t_prev = now
        arrivals.append(now)
        fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
        frames += 1
        poll_mav()

        if K is None:
            h_img, w = frame.shape[:2]
            fx = (w / 2) / math.tan(args.hfov / 2) * args.zoom
            ccx, ccy = w / 2, h_img / 2
            s = args.disp_w / w
            cv2.resizeWindow(win, args.disp_w, PANEL_H + int(h_img * s))
            K = True
            print(f"{w}x{h_img} fx={fx:.1f}")

        kf.predict(now)

        # ---- tespit ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect(gray)
        found = None
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if args.id < 0 or int(i) == args.id:
                    found = (int(i), c.reshape(4, 2))
                    break
        if found:
            hits += 1

        h_raw = -st["d"]                # yukseklik: EKF z (dock = home yuksekliginde varsayim)
        h = max(0.3, h_raw)             # olcek hesabi icin alt sinirli
        marker_id, ax_deg, ay_deg = None, 0.0, 0.0

        if found:
            marker_id, pts = found
            cx, cy = pts.mean(axis=0)
            ax, ay = math.atan2(cx - ccx, fx), math.atan2(cy - ccy, fx)
            ax_deg, ay_deg = math.degrees(ax), math.degrees(ay)
            yaw_img = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
            yaw_f = yaw_img if yaw_f is None else wrap(yaw_f + 0.3 * wrap(yaw_img - yaw_f))

            if args.precland:
                send_landing_target(ax, ay)     # kontrol blogunun disinda: LAND'de de akmaya devam eder

            if st["pos_ok"]:
                rel = los_to_level_xy(ax, ay, h)
                if rel:
                    fwd, right = rel
                    y = st["yaw"]
                    tn = st["n"] + fwd * math.cos(y) - right * math.sin(y)
                    te = st["e"] + fwd * math.sin(y) + right * math.cos(y)
                    kf.update(now, tn, te, args.noise * h + 0.02)
        else:
            yaw_f = None

        # ---- hedef durumu ----
        lost_lim = args.final_lost_s if seq.final else args.lost_s
        tracking = kf.ok and (now - kf.t_last < lost_lim) and st["pos_ok"]
        err = None
        rx = ry = None
        en = ee = 0.0
        tn = te = 0.0
        yaw_rate_cmd = None
        if tracking:
            tn, te = kf.pos()
            en, ee = tn - st["n"], te - st["e"]
            err = math.hypot(en, ee)
            cyaw, syaw = math.cos(st["yaw"]), math.sin(st["yaw"])
            rx = en * cyaw + ee * syaw         # X: ileri (+) / geri (-)
            ry = -en * syaw + ee * cyaw        # Y: sag (+) / sol (-)

        # ---- inis / disarm sirasi ----
        act = seq.update(now, running, st["armed"], st["landed"], h_raw, st["vd"], err, tracking)
        if act["land"]:
            print("LAND moduna geciliyor")
            m.set_mode("LAND")
        if act["disarm"]:
            send_disarm(act["disarm"] == "force")
            print(f"DISARM komutu ({act['disarm']}) deneme {seq.tries}")
        if seq.phase != prev_phase:
            print(f"asama: {PHASE_TR[prev_phase]} -> {PHASE_TR[seq.phase]}  {seq.msg}")
            prev_phase = seq.phase
        if seq.phase == "DONE" and running:
            running = False
            cmd_vn = cmd_ve = cmd_vz = 0.0
            print(seq.msg)

        # ---- kontrol ----
        guided = st["mode"] == MODE_GUIDED
        if running and guided and seq.phase not in ("LAND", "DONE"):
            if seq.phase == "TOUCH":
                # yerde: yan hareket yok, hafif asagi baski (land detector icin), disarm denenir
                cmd_vn = cmd_ve = 0.0
                cmd_vz = args.vfinal
                send_vel_ned(0.0, 0.0, cmd_vz, 0.0 if yaw_align else None)
            elif tracking:
                vn, ve = args.kp * en, args.kp * ee
                sp_ = math.hypot(vn, ve)
                if sp_ > args.vmax:
                    vn, ve = vn * args.vmax / sp_, ve * args.vmax / sp_
                # ivme sinirli yumusatma
                dv = args.amax * dt
                cmd_vn += float(np.clip(vn - cmd_vn, -dv, dv))
                cmd_ve += float(np.clip(ve - cmd_ve, -dv, dv))

                cmd_vz = 0.0
                if seq.phase in ("DESCEND", "FINAL"):
                    cmd_vz = float(np.clip(0.3 * h, args.vfinal, 0.6))

                if yaw_align and yaw_f is not None and abs(yaw_f) > math.radians(2):
                    lim = math.radians(args.yawmax)
                    yaw_rate_cmd = float(np.clip(args.kyaw * yaw_f, -lim, lim))
                elif yaw_align:
                    yaw_rate_cmd = 0.0

                send_vel_ned(cmd_vn, cmd_ve, cmd_vz, yaw_rate_cmd)
            else:
                cmd_vn = cmd_ve = cmd_vz = 0.0
                send_vel_ned(0.0, 0.0, 0.0, 0.0 if yaw_align else None)

        # ---- HUD ----
        disp = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        cv2.drawMarker(disp, (int(ccx * s), int(ccy * s)), RED, cv2.MARKER_CROSS, 20, 1)
        if found:
            cv2.aruco.drawDetectedMarkers(
                disp, [(found[1] * s).reshape(1, 4, 2).astype(np.float32)],
                np.array([[found[0]]], dtype=np.int32))
            c = found[1].mean(axis=0)
            cv2.line(disp, (int(ccx * s), int(ccy * s)), (int(c[0] * s), int(c[1] * s)), WHITE, 1)

        mode_name = MODE_NAMES.get(st["mode"], str(st["mode"]))
        if found:
            lock_item = (f"Marker : KILITLI  id {marker_id}", GREEN)
        elif tracking:
            lock_item = ("Marker : KAYIP (KF tahmini)", WHITE)
        else:
            lock_item = ("Marker : YOK", RED)

        if st["armed"] is None:
            arm_item = ("Arm : ---", WHITE)
        elif st["armed"]:
            arm_item = ("Arm : ARMED", GREEN)
        else:
            arm_item = ("Arm : DISARMED", WHITE)

        row1 = [
            ("Durum : CALISIYOR" if running else "Durum : DURDU", GREEN if running else RED),
            (f"Mod : {mode_name}", GREEN if guided else RED),
            arm_item,
            lock_item,
            (f"FPS : {fps:.0f}", WHITE),
            (f"Yukseklik : {h_raw:.2f} m", WHITE),
        ]

        if rx is not None:
            row2 = [
                (f"X (ileri) : {rx:+.2f} m", GREEN if abs(rx) < args.tol else WHITE),
                (f"Y (sag) : {ry:+.2f} m", GREEN if abs(ry) < args.tol else WHITE),
                (f"Mesafe : {err:.2f} m", GREEN if err < args.tol else WHITE),
            ]
        else:
            row2 = [("X (ileri) : ---", WHITE), ("Y (sag) : ---", WHITE), ("Mesafe : ---", WHITE)]

        if yaw_align:
            yaw_txt = ("Yaw : ACIK" + (f" ({math.degrees(yaw_f):+.0f} deg)" if yaw_f is not None else ""), GREEN)
        else:
            yaw_txt = ("Yaw : KAPALI (y)", WHITE)
        phase_col = GREEN if seq.phase == "DONE" else (RED if seq.failed else WHITE)
        if args.precland:
            inis_tr = "ArduPilot precland"
        elif args.land_mode:
            inis_tr = "LAND modu"
        else:
            inis_tr = "GUIDED + disarm"
        row3 = [
            (f"Asama : {PHASE_TR[seq.phase]}", phase_col),
            (f"Inis : {inis_tr}" + (" (kapali)" if args.no_descend else ""), WHITE),
            (f"Aci : {ax_deg:+.1f}, {ay_deg:+.1f} deg" if found else "Aci : ---", WHITE),
            (f"Hiz N/E/D : {cmd_vn:+.2f} {cmd_ve:+.2f} {cmd_vz:+.2f}", WHITE),
            yaw_txt,
        ]
        msg_col = GREEN if seq.phase == "DONE" else (RED if seq.failed else WHITE)
        row4 = [(seq.msg if seq.msg else "-", msg_col)]

        disp = np.vstack([build_panel(disp.shape[1], [
            (row1, fs, TH, ROW_H), (row2, fs * BIG, TH_BIG, BIG_H),
            (row3, fs, TH, ROW_H), (row4, fs, TH, MSG_H)]), disp])
        cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" "):
            running = not running
            cmd_vn, cmd_ve, cmd_vz = st["vn"], st["ve"], 0.0
            if running:
                seq.reset()
                prev_phase = seq.phase
                print("START")
            else:
                print("PAUSE")
                if guided:
                    send_vel_ned(0.0, 0.0, 0.0, 0.0 if yaw_align else None)
        elif key == ord("y"):
            yaw_align = not yaw_align
            if not yaw_align and running and guided:
                send_vel_ned(cmd_vn, cmd_ve, cmd_vz, None)

        if now - last_print > 1.0:
            last_print = now
            if tracking:
                print(f"h={h_raw:.2f} hedef=({tn:+.2f},{te:+.2f}) hata={err:.2f}m "
                      f"mode={mode_name} asama={PHASE_TR[seq.phase]} run={running}")
finally:
    cap.release()
    cv2.destroyAllWindows()
    print(f"kare: {frames}, tespit: {hits}")
