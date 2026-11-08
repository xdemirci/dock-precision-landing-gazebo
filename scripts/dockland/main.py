#!/usr/bin/env python3
"""
ArUco ile dock ortalama + inis (kendi PosVel KF, ArduPilot precland KULLANILMAZ).

    python3 main.py --conn udp:127.0.0.1:14550
    python3 main.py                                   # rangefinder'siz: yukseklik PnP'den ogrenilir (--h-src pnp)
    python3 main.py --h-src rng                       # lidar varsa
    python3 main.py --h-src ekf --dock-h 1.4          # sabit dock yuksekligi
    python3 main.py --yaw-align --vmax 0.3

Tuslar:  SPACE = start/pause   y = marker'a yonelme ac/kapa   q = cikis
Akis:    HIZALA -> ALCAL -> SON INIS -> TEMAS -> DISARM dogrula  (--land-mode: son asamada LAND)
"""
import math
import time
from collections import deque

import cv2

import pose
from camera import Camera
from config import parse_args
from controller import Controller
from detector import MarkerDetector
from estimator import TargetFilter
from height import MarkerHeight
from hud import Hud
from mission import PHASE_TR, LandingSequencer
from vehicle import Vehicle


def main():
    cfg = parse_args()
    cam_in = Camera(cfg)
    veh = Vehicle(cfg.conn)
    detector, kf = MarkerDetector(cfg), TargetFilter()
    seq, ctl, hud = LandingSequencer(cfg), Controller(cfg), Hud(cfg)
    yaw_f = pose.YawFilter()
    mh = MarkerHeight(cfg)

    win = "Dock Landing"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cam, scale = None, 1.0
    running, yaw_align = False, cfg.yaw_align
    arrivals = deque(maxlen=30)
    t_prev, last_print, prev_phase = None, 0.0, seq.phase
    frames = locked_n = 0

    try:
        while True:
            frame, _ = cam_in.read()
            veh.poll()
            if frame is None:
                continue
            now = time.monotonic()
            dt = (now - t_prev) if t_prev else 0.05
            t_prev = now
            arrivals.append(now)
            fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
            frames += 1

            if cam is None:
                h_img, w = frame.shape[:2]
                cam = pose.CamModel(w, h_img, cfg.hfov, cfg.zoom, cfg.calib)
                scale = cfg.disp_w / w
                cv2.resizeWindow(win, cfg.disp_w, hud.panel_h + int(h_img * scale))
                print(f"{w}x{h_img} fx={cam.K[0, 0]:.1f}")

            kf.predict(now)

            # ---- tespit + PnP ----
            det = detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            ax = ay = 0.0
            if det:
                pose.solve_pnp(det, cam, cfg.marker)
                det.good = pose.is_good(det, cfg)
                ax, ay = cam.angles(det.center)
            h, h_src = mh.update(veh, det, now)
            hs = max(0.3, h)                                  # olcek hesabi icin alt sinirli

            # ---- olcum -> KF (sadece kilitli + PnP kalite kapisindan gecen) ----
            if det and det.locked and det.good:
                locked_n += 1
                yaw_f.update(det.yaw)
                rel = pose.los_to_level_xy(ax, ay, hs, veh.roll, veh.pitch) if veh.pos_ok else None
                if rel:
                    fwd, right = rel
                    cy, sy = math.cos(veh.yaw), math.sin(veh.yaw)
                    kf.update(now, veh.n + fwd * cy - right * sy, veh.e + fwd * sy + right * cy,
                              cfg.noise * hs + 0.02)
            else:
                yaw_f.reset()

            # ---- hedef durumu ----
            lost_lim = cfg.final_lost_s if seq.final else cfg.lost_s
            tracking = kf.ok and (now - kf.t_last < lost_lim) and veh.pos_ok
            err = rx = ry = None
            en = ee = 0.0
            if tracking:
                tn, te = kf.pos()
                en, ee = tn - veh.n, te - veh.e
                err = math.hypot(en, ee)
                cy, sy = math.cos(veh.yaw), math.sin(veh.yaw)
                rx, ry = en * cy + ee * sy, -en * sy + ee * cy      # govde: ileri (+), sag (+)

            # ---- inis / disarm sirasi ----
            act = seq.update(now, running, veh.armed, veh.landed, h, veh.vd, err, tracking)
            if act["land"]:
                print("LAND moduna geciliyor")
                veh.set_mode("LAND")
            if act["disarm"]:
                veh.send_disarm(act["disarm"] == "force")
                print(f"DISARM komutu ({act['disarm']}) deneme {seq.tries}")
            if seq.phase != prev_phase:
                print(f"asama: {PHASE_TR[prev_phase]} -> {PHASE_TR[seq.phase]}  {seq.msg}")
                prev_phase = seq.phase
            if seq.phase == "DONE" and running:
                running = False
                ctl.reset()
                print(seq.msg)

            # ---- kontrol ----
            if running and veh.guided and seq.phase not in ("LAND", "DONE"):
                veh.send_vel_ned(*ctl.step(dt, seq.phase, tracking, en, ee, h,
                                           yaw_f.value if yaw_align else None, yaw_align))

            # ---- ekran ----
            disp = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            hud.overlay(disp, det, scale, cam)
            disp = hud.render(disp, dict(
                det=det, tracking=tracking, running=running, guided=veh.guided, mode_name=veh.mode_name,
                armed=veh.armed, fps=fps, h=h, h_src=h_src, rx=rx, ry=ry, err=err,
                phase_tr=PHASE_TR[seq.phase], seq_msg=seq.msg, seq_failed=seq.failed,
                seq_done=seq.phase == "DONE", cmd=(ctl.vn, ctl.ve, ctl.vz), yaw_align=yaw_align,
                yaw_err=yaw_f.value, alt=-veh.d, m_alt=mh.m_alt, rng=veh.rng if veh.rng_ok(now) else None, ax_deg=math.degrees(ax), ay_deg=math.degrees(ay)))
            cv2.imshow(win, disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                running = not running
                ctl.reset(veh.vn, veh.ve)
                if running:
                    seq.reset()
                    prev_phase = seq.phase
                    print("START")
                else:
                    print("PAUSE")
                    if veh.guided:
                        veh.send_vel_ned(0.0, 0.0, 0.0, 0.0 if yaw_align else None)
            elif key == ord("y"):
                yaw_align = not yaw_align
                if not yaw_align and running and veh.guided:
                    veh.send_vel_ned(ctl.vn, ctl.ve, ctl.vz, None)

            if tracking and now - last_print > 1.0:
                last_print = now
                tn, te = kf.pos()
                print(f"h={h:.2f}({h_src}) hedef=({tn:+.2f},{te:+.2f}) hata={err:.2f}m "
                      f"mode={veh.mode_name} asama={PHASE_TR[seq.phase]} run={running}")
    finally:
        cam_in.close()
        cv2.destroyAllWindows()
        print(f"kare: {frames}, kilitli+PnP gecen: {locked_n}, KF reddedilen: {kf.rejected}")


if __name__ == "__main__":
    main()
