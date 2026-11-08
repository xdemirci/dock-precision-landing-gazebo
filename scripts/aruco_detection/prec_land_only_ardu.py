#!/usr/bin/env python3
"""
SADECE ArduPilot precland: ArUco -> LANDING_TARGET. Kestirim, ortalama ve inis ArduPilot'ta.
(Kendi KF'i, hiz komutu, disarm vs. YOK.)

Tuslar:  SPACE = LANDING_TARGET gonderimi ac/kapa   l = LAND moduna gec   q = cikis

ArduPilot parametreleri (sonra reboot):
    PLND_ENABLED 1, PLND_TYPE 1, PLND_EST_TYPE 1, PLND_LAG 0.15, PLND_STRICT 1
    RNGFND1_TYPE 100 (rangefinder dock'u gormeli)

    python3 precland_ardu.py --conn udp:127.0.0.1:14550
    python3 precland_ardu.py --flip-y          # LAND'de ters kayarsa
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
ap.add_argument("--hfov", type=float, default=1.151917)
ap.add_argument("--zoom", type=float, default=1.0)
ap.add_argument("--id", type=int, default=-1)
ap.add_argument("--flip-x", action="store_true", help="angle_x isaretini cevir")
ap.add_argument("--flip-y", action="store_true", help="angle_y isaretini cevir")
args = ap.parse_args()

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
    "rtpjitterbuffer latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false")
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# ---------------- MAVLink ----------------
m = mavutil.mavlink_connection(args.conn)
m.wait_heartbeat()
print("heartbeat ok")
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000, 0, 0, 0, 0, 0)

st = dict(mode=-1, armed=None, rng=None, t_rng=0.0, text="-")
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
              6: "RTL", 9: "LAND", 16: "POSHOLD"}


def poll_mav():
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if (t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            st["mode"] = msg.custom_mode
            st["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "DISTANCE_SENSOR" and msg.orientation == 25:
            st["rng"] = msg.current_distance / 100.0
            st["t_rng"] = time.monotonic()
        elif t == "STATUSTEXT":
            txt = msg.text.decode(errors="ignore") if isinstance(msg.text, bytes) else str(msg.text)
            st["text"] = txt.strip("\x00 ")
            print("AP:", st["text"])


def send_landing_target(ax, ay):
    # AC_PrecLand_MAVLink: vec = (-tan(angle_y), tan(angle_x), 1) BODY_FRD; distance=0 -> rangefinder
    m.mav.landing_target_send(int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                              -ax if args.flip_x else ax, -ay if args.flip_y else ay,
                              0.0, 0.0, 0.0)


# ---------------- HUD ----------------
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, WHITE, RED = (0, 255, 0), (255, 255, 255), (60, 60, 255)
fs = max(0.42, args.disp_w / 2300)
PAD = int(fs * 14)
ROW_H = int(fs * 50)
PANEL_H = 2 * PAD + 3 * ROW_H


def draw_row(img, y_top, items):
    W = img.shape[1] - 2 * PAD

    def measure(sc):
        return [cv2.getTextSize(t, FONT, sc, 1)[0][0] for t, _ in items]

    scale = fs
    gap = int(36 * scale)
    total = sum(measure(scale)) + gap * (len(items) - 1)
    if total > W:
        scale *= W / total
        gap = int(36 * scale)
    ws = measure(scale)
    x = PAD
    base = y_top + int(ROW_H * 0.72)
    for k, (t, col) in enumerate(items):
        cv2.putText(img, t, (x, base), FONT, scale, col, 1, cv2.LINE_AA)
        x += ws[k] + gap // 2
        if k < len(items) - 1:
            cv2.line(img, (x, y_top + int(ROW_H * 0.2)), (x, y_top + int(ROW_H * 0.8)), (90, 90, 90), 1)
        x += gap // 2


def build_panel(width, rows):
    panel = np.full((PANEL_H, width, 3), 28, dtype=np.uint8)
    y = PAD
    for k, items in enumerate(rows):
        draw_row(panel, y, items)
        y += ROW_H
        if k < len(rows) - 1:
            cv2.line(panel, (PAD, y), (width - PAD, y), (70, 70, 70), 1)
    return panel


# ---------------- Ana dongu ----------------
win = "ArduPilot PrecLand"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
K = None
s = 1.0
fx = ccx = ccy = 0.0
sending = False
sent = 0
sent_t = deque(maxlen=40)
arrivals = deque(maxlen=30)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        now = time.monotonic()
        arrivals.append(now)
        fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
        poll_mav()

        if K is None:
            h_img, w = frame.shape[:2]
            fx = (w / 2) / math.tan(args.hfov / 2) * args.zoom
            ccx, ccy = w / 2, h_img / 2
            s = args.disp_w / w
            cv2.resizeWindow(win, args.disp_w, PANEL_H + int(h_img * s))
            K = True
            print(f"{w}x{h_img} fx={fx:.1f}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect(gray)
        found = None
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if args.id < 0 or int(i) == args.id:
                    found = (int(i), c.reshape(4, 2))
                    break

        ax_deg = ay_deg = 0.0
        if found:
            cx, cy = found[1].mean(axis=0)
            ax, ay = math.atan2(cx - ccx, fx), math.atan2(cy - ccy, fx)
            ax_deg, ay_deg = math.degrees(ax), math.degrees(ay)
            if sending:
                send_landing_target(ax, ay)
                sent += 1
                sent_t.append(now)

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
        rate = (len(sent_t) - 1) / (sent_t[-1] - sent_t[0]) if len(sent_t) > 1 and now - sent_t[-1] < 1 else 0.0
        rng_ok = st["rng"] is not None and now - st["t_rng"] < 1.0
        arm_item = (("Arm : ARMED", GREEN) if st["armed"] else ("Arm : DISARMED", WHITE)) \
            if st["armed"] is not None else ("Arm : ---", WHITE)

        row1 = [
            ("Gonderim : ACIK" if sending else "Gonderim : DURDU (SPACE)", GREEN if sending else RED),
            (f"Mod : {mode_name}", GREEN if st["mode"] == 9 else WHITE),
            arm_item,
            (f"Marker : KILITLI  id {found[0]}", GREEN) if found else ("Marker : YOK", RED),
            (f"FPS : {fps:.0f}", WHITE),
        ]
        row2 = [
            (f"Rangefinder : {st['rng']:.2f} m" if rng_ok else "Rangefinder : ---", WHITE if rng_ok else RED),
            (f"Aci : {ax_deg:+.1f}, {ay_deg:+.1f} deg" if found else "Aci : ---", WHITE),
            (f"LANDING_TARGET : {sent}  ({rate:.0f} Hz)", GREEN if rate > 0 else WHITE),
        ]
        low = st["text"].lower()
        txt_col = RED if "lost" in low else (GREEN if ("found" in low or "init" in low) else WHITE)
        row3 = [(f"AP : {st['text']}", txt_col)]

        disp = np.vstack([build_panel(disp.shape[1], [row1, row2, row3]), disp])
        cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" "):
            sending = not sending
            print("GONDERIM", "ACIK" if sending else "DURDU")
        elif key == ord("l"):
            print("LAND moduna geciliyor")
            m.set_mode("LAND")
finally:
    cap.release()
    cv2.destroyAllWindows()
    print(f"gonderilen LANDING_TARGET: {sent}")
