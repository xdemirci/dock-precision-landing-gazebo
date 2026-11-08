"""Video girisi: arka planda surekli okur, ana donguye hep EN SON kareyi verir (Randy'nin image_capture mantigi)."""
import threading
import time

import cv2


def build_pipeline(port, lowlat=False):
    jb = "rtpjitterbuffer latency=0 ! " if lowlat else ""
    return (
        f"udpsrc port={port} caps=\"application/x-rtp, media=(string)video, "
        "clock-rate=(int)90000, encoding-name=(string)H264\" ! "
        f"{jb}rtph264depay ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false")


class Camera:
    def __init__(self, cfg):
        self.cap = cv2.VideoCapture(build_pipeline(cfg.port, cfg.lowlat), cv2.CAP_GSTREAMER)
        self._cv = threading.Condition()
        self._frame, self._t = None, 0.0
        self._id = self._last_id = 0
        self._run = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while self._run:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._cv:
                self._frame, self._t = f, time.monotonic()
                self._id += 1
                self._cv.notify_all()

    def read(self, timeout=1.0):
        """Yeni kare gelene kadar bekler. -> (frame, varis_zamani) | (None, 0.0)"""
        with self._cv:
            if not self._cv.wait_for(lambda: self._id != self._last_id, timeout):
                return None, 0.0
            self._last_id = self._id
            return self._frame, self._t

    def close(self):
        self._run = False
        self._th.join(timeout=1.0)
        self.cap.release()
