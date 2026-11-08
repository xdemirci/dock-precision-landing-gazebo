"""Ekran: video ustunde, tam genislikte bilgi paneli. Yazilar --disp-w ile olceklenir."""
import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, WHITE, RED = (0, 255, 0), (255, 255, 255), (60, 60, 255)


class Hud:
    N_ROWS = 5

    def __init__(self, cfg):
        self.cfg = cfg
        self.fs = max(0.42, cfg.disp_w / 2300)
        self.pad = int(self.fs * 14)
        self.row_h = int(self.fs * 50)
        self.panel_h = 2 * self.pad + self.N_ROWS * self.row_h

    # ---- yardimcilar ----
    def _row(self, img, y_top, items):
        fs, W = self.fs, img.shape[1] - 2 * self.pad

        def widths(sc):
            return [cv2.getTextSize(t, FONT, sc, 1)[0][0] for t, _ in items]

        scale, gap = fs, int(36 * fs)
        total = sum(widths(scale)) + gap * (len(items) - 1)
        if total > W:                              # tasarsa otomatik kucult
            scale *= W / total
            gap = int(36 * scale)
        ws, x = widths(scale), self.pad
        base = y_top + int(self.row_h * 0.72)
        for k, (t, col) in enumerate(items):
            cv2.putText(img, t, (x, base), FONT, scale, col, 1, cv2.LINE_AA)
            x += ws[k] + gap // 2
            if k < len(items) - 1:
                cv2.line(img, (x, y_top + int(self.row_h * 0.2)), (x, y_top + int(self.row_h * 0.8)), (90, 90, 90), 1)
            x += gap // 2

    def overlay(self, disp, det, scale, cam):
        c = (int(cam.cx * scale), int(cam.cy * scale))
        cv2.drawMarker(disp, c, RED, cv2.MARKER_CROSS, 20, 1)
        if det is not None:
            cv2.aruco.drawDetectedMarkers(
                disp, [(det.corners * scale).reshape(1, 4, 2).astype(np.float32)],
                np.array([[det.id]], dtype=np.int32))
            cv2.line(disp, c, (int(det.center[0] * scale), int(det.center[1] * scale)), WHITE, 1)

    # ---- ana cizim ----
    def render(self, disp, i):
        """i: bilgi sozlugu (main.py doldurur). -> panel + video"""
        c, det = self.cfg, i["det"]
        rows = []

        if det and det.locked and det.good:
            marker = (f"Marker : KILITLI  id {det.id}", GREEN)
        elif det and det.locked:
            marker = ("Marker : ZAYIF TESPIT (PnP red)", WHITE)
        elif det:
            marker = ("Marker : DOGRULANIYOR", WHITE)
        elif i["tracking"]:
            marker = ("Marker : KAYIP (KF tahmini)", WHITE)
        else:
            marker = ("Marker : YOK", RED)
        arm = ("Arm : ---", WHITE) if i["armed"] is None else (
            ("Arm : ARMED", GREEN) if i["armed"] else ("Arm : DISARMED", WHITE))
        rows.append([
            ("Durum : CALISIYOR" if i["running"] else "Durum : DURDU", GREEN if i["running"] else RED),
            (f"Mod : {i['mode_name']}", GREEN if i["guided"] else RED),
            arm, marker,
            (f"FPS : {i['fps']:.0f}", WHITE),
            (f"Yukseklik : {i['h']:.2f} m ({i['h_src']})", WHITE)])

        if i["rx"] is not None:
            xy = [
                (f"X (ileri) : {i['rx']:+.2f} m", GREEN if abs(i["rx"]) < c.tol else WHITE),
                (f"Y (sag) : {i['ry']:+.2f} m", GREEN if abs(i["ry"]) < c.tol else WHITE),
                (f"Mesafe : {i['err']:.2f} m", GREEN if i["err"] < c.tol else WHITE)]
        else:
            xy = [("X (ileri) : ---", WHITE), ("Y (sag) : ---", WHITE), ("Mesafe : ---", WHITE)]
        # kalibrasyon icin: EKF irtifasi ve ogrenilen marker irtifasi (home'a gore)
        rows.append(xy + [(f"Irtifa (EKF) : {i['alt']:.2f} m", WHITE), (f"Marker alt : {i['m_alt']:+.2f} m", WHITE)])

        yaw = ("Yaw : KAPALI (y)", WHITE)
        if i["yaw_align"]:
            yaw = ("Yaw : ACIK" + (f" ({np.degrees(i['yaw_err']):+.0f} deg)" if i["yaw_err"] is not None else ""), GREEN)
        phase_col = GREEN if i["seq_done"] else (RED if i["seq_failed"] else WHITE)
        inis = "LAND modu" if c.land_mode else "GUIDED + disarm"
        rows.append([
            (f"Asama : {i['phase_tr']}", phase_col),
            (f"Inis : {inis}" + (" (kapali)" if c.no_descend else ""), WHITE),
            (f"Hiz N/E/D : {i['cmd'][0]:+.2f} {i['cmd'][1]:+.2f} {i['cmd'][2]:+.2f}", WHITE),
            yaw])

        if det:
            rows.append([
                (f"Aci : {i['ax_deg']:+.1f}, {i['ay_deg']:+.1f} deg", WHITE),
                (f"PnP z : {det.tvec[2]:.2f} m" if det.pnp_ok else "PnP : ---", WHITE),
                (f"Reproj : {det.reproj:.2f} px", GREEN if det.good else RED),
                (f"Tilt : {det.tilt:.0f} deg", WHITE),
                (f"Boy : {det.side_px:.0f} px", WHITE),
                (f"Rng : {i['rng']:.2f} m" if i["rng"] is not None else "Rng : ---", WHITE)])
        else:
            rows.append([("Aci : ---", WHITE), ("PnP : ---", WHITE)])

        rows.append([(i["seq_msg"] or "-", phase_col)])

        panel = np.full((self.panel_h, disp.shape[1], 3), 28, dtype=np.uint8)
        y = self.pad
        for k, items in enumerate(rows):
            self._row(panel, y, items)
            y += self.row_h
            if k < len(rows) - 1:
                cv2.line(panel, (self.pad, y), (disp.shape[1] - self.pad, y), (70, 70, 70), 1)
        return np.vstack([panel, disp])
