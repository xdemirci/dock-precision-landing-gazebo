"""Marker tespiti + basit saglamlik filtreleri (geometri, ID, ardisik-kare kilidi, golge duzeltme)."""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    id: int
    corners: np.ndarray            # (4,2) TL, TR, BR, BL (px)
    center: tuple                  # (cx, cy) px
    side_px: float
    locked: bool = False           # min_hits ardisik karedir goruluyor
    deshadowed: bool = False       # golge duzeltmeyle bulundu
    # --- pose.solve_pnp doldurur ---
    pnp_ok: bool = False
    good: bool = False             # PnP kalite kapilarindan gecti
    tvec: np.ndarray = None        # marker merkezi, kamera cercevesi (m)
    reproj: float = 0.0            # px
    tilt: float = 0.0              # derece
    yaw: float = 0.0               # marker yaw, goruntu icinde (rad)


class MarkerDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        name = "DICT_" + cfg.dict.upper()
        if not hasattr(cv2.aruco, name):
            raise SystemExit(f"bilinmeyen ArUco sozlugu: {cfg.dict}")
        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        params = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create")
                  else cv2.aruco.DetectorParameters())
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 203
        params.adaptiveThreshWinSizeStep = 20
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detect = cv2.aruco.ArucoDetector(dic, params).detectMarkers
        else:
            self._detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=params)[:3]
        self.streak = 0

    @staticmethod
    def _deshadow(gray):
        """Arka plan parlakligini morfolojik kapamayla kestirip goruntuyu ona boler: sert golge kenarlari
        marker'in siyah cercevesine karisip tespiti bozar. (1/4 cozunurlukte ~14 ms; sadece normal tespit
        basarisizsa calisir.)"""
        small = cv2.resize(gray, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))      # ~164 px: siyah cerceveden kalin
        bg = cv2.morphologyEx(small, cv2.MORPH_CLOSE, k)
        bg = cv2.resize(bg, (gray.shape[1], gray.shape[0]), interpolation=cv2.INTER_LINEAR)
        return np.clip(gray.astype(np.float32) / np.maximum(bg, 1) * 255, 0, 255).astype(np.uint8)

    def process(self, gray):
        """-> Detection | None"""
        det = self._best(gray)
        if det is None:
            det = self._best(self._deshadow(gray))
            if det is not None:
                det.deshadowed = True
        if det is None:
            self.streak = 0
            return None
        self.streak += 1
        det.locked = self.streak >= self.cfg.min_hits
        return det

    def _best(self, gray):
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return None
        best = None
        for c, i in zip(corners, ids.flatten()):
            if self.cfg.id >= 0 and int(i) != self.cfg.id:
                continue
            p = c.reshape(4, 2).astype(np.float64)
            sides = [np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]
            side = float(np.mean(sides))
            # cok kucuk / cok carpik aday = buyuk ihtimalle yanlis pozitif
            if side < self.cfg.min_side or max(sides) / max(min(sides), 1e-6) > 2.5:
                continue
            if best is None or side > best.side_px:
                best = Detection(int(i), p, (float(p[:, 0].mean()), float(p[:, 1].mean())), side)
        return best
