"""Hiz komutu: P kontrol + hiz siniri + ivme siniri (yumusatma) + alcalma + (istege bagli) yaw hizalama."""
import math

import numpy as np


class Controller:
    def __init__(self, cfg):
        self.cfg = cfg
        self.vn = self.ve = self.vz = 0.0

    def reset(self, vn=0.0, ve=0.0):
        self.vn, self.ve, self.vz = vn, ve, 0.0

    def step(self, dt, phase, tracking, en, ee, h, yaw_err, yaw_align):
        """-> (vn, ve, vz, yaw_rate | None). en/ee: arac->hedef NED hatasi (m); h: marker'a yukseklik (m);
        yaw_err: filtrelenmis marker yaw (rad) | None."""
        c = self.cfg
        zero_yaw = 0.0 if yaw_align else None

        if phase == "TOUCH":                    # yerde: yan hareket yok, hafif asagi baski
            self.vn = self.ve = 0.0
            self.vz = c.vfinal
            return 0.0, 0.0, self.vz, zero_yaw
        if not tracking:
            self.vn = self.ve = self.vz = 0.0
            return 0.0, 0.0, 0.0, zero_yaw

        vn, ve = c.kp * en, c.kp * ee
        sp = math.hypot(vn, ve)
        if sp > c.vmax:
            vn, ve = vn * c.vmax / sp, ve * c.vmax / sp
        dv = c.amax * dt
        self.vn += float(np.clip(vn - self.vn, -dv, dv))
        self.ve += float(np.clip(ve - self.ve, -dv, dv))

        self.vz = 0.0
        if phase in ("DESCEND", "FINAL"):
            self.vz = float(np.clip(0.3 * max(0.3, h), c.vfinal, 0.6))

        yaw_rate = zero_yaw
        if yaw_align and yaw_err is not None and abs(yaw_err) > math.radians(2):
            lim = math.radians(c.yawmax)
            yaw_rate = float(np.clip(c.kyaw * yaw_err, -lim, lim))
        return self.vn, self.ve, self.vz, yaw_rate
