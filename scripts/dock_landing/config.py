"""Tum ayarlar tek yerde. Diger moduller sadece cfg.<alan> okur."""
import argparse


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="ArUco dock ortalama + inis (kendi PosVel KF)")

    g = ap.add_argument_group("baglanti / video")
    g.add_argument("--conn", default="udp:127.0.0.1:14550")
    g.add_argument("--port", type=int, default=5600)
    g.add_argument("--lowlat", action="store_true", help="GStreamer rtpjitterbuffer latency=0")
    g.add_argument("--disp-w", type=int, default=1080, help="pencere genisligi (px)")

    g = ap.add_argument_group("kamera / marker")
    g.add_argument("--hfov", type=float, default=1.151917, help="yatay FOV (rad)")
    g.add_argument("--zoom", type=float, default=1.0)
    g.add_argument("--calib", default=None, help="kalibrasyon .npz (camera_matrix/distortion_coeffs veya K/D)")
    g.add_argument("--marker", type=float, default=0.4165, help="siyah cerceve dis kenari (m)")
    g.add_argument("--id", type=int, default=-1, help="sadece bu ID (-1: hepsi)")
    g.add_argument("--dict", default="4X4_50", help="ArUco sozlugu: 4X4_50, 5X5_100, APRILTAG_36H11 ...")

    g = ap.add_argument_group("tespit saglamligi")
    g.add_argument("--min-hits", type=int, default=2, help="kilitlenmek icin ardisik kare")
    g.add_argument("--min-side", type=float, default=12.0, help="min marker kenari (px)")
    g.add_argument("--max-reproj", type=float, default=2.0, help="PnP yeniden-izdusum hata esigi (px)")
    g.add_argument("--max-tilt", type=float, default=60.0, help="marker egim esigi (derece)")

    g = ap.add_argument_group("dock / yukseklik")
    g.add_argument("--dock-h", type=float, default=0.0,
                   help="marker duzleminin EKF orijinine (home) gore yuksekligi (m)")
    g.add_argument("--h-scale", type=float, default=1.0,
                   help="PnP derinlik duzeltme carpani (gercek/PnP). Kalibrasyon icin: lidar veya bilinen yukseklikle olc")
    g.add_argument("--h-src", choices=("ekf", "rng", "pnp"), default="pnp",
                   help="marker'a yukseklik: pnp (PnP'den ogrenir, rangefinder yok), rng (DISTANCE_SENSOR), ekf (sabit --dock-h)")

    g = ap.add_argument_group("ortalama")
    g.add_argument("--kp", type=float, default=0.5, help="konum hatasi -> hiz (1/s)")
    g.add_argument("--vmax", type=float, default=0.5, help="max yatay hiz m/s")
    g.add_argument("--amax", type=float, default=0.6, help="max yatay ivme m/s^2")
    g.add_argument("--tol", type=float, default=0.15, help="ortalandi esigi (m)")
    g.add_argument("--noise", type=float, default=0.02, help="olcum sigma = noise * yukseklik")
    g.add_argument("--lost-s", type=float, default=1.0, help="marker kaybi: KF ile devam suresi (s)")
    g.add_argument("--yaw-align", action="store_true", help="baslangicta marker'a yonelme acik")
    g.add_argument("--kyaw", type=float, default=0.8, help="yaw hata(rad) -> yaw hizi (rad/s)")
    g.add_argument("--yawmax", type=float, default=30.0, help="max yaw hizi deg/s")

    g = ap.add_argument_group("inis / disarm")
    g.add_argument("--land-mode", action="store_true", help="son asamada LAND moduna gec (disarm'i ArduPilot yapar)")
    g.add_argument("--no-descend", "--no-land", dest="no_descend", action="store_true", help="sadece ortala, inme")
    g.add_argument("--no-disarm", action="store_true", help="yere inince disarm gonderme")
    g.add_argument("--force-disarm", action="store_true", help="disarm reddedilirse zorla (21196)")
    g.add_argument("--final-h", type=float, default=2.0, help="bu yukseklik altinda marker kaybolsa da KF ile dik inis (m, marker ustu)")
    g.add_argument("--final-lost-s", type=float, default=20.0, help="son inista KF ile devam suresi (s)")
    g.add_argument("--vfinal", type=float, default=0.2, help="min inis hizi m/s")
    g.add_argument("--touch-h", type=float, default=0.6, help="yedek temas tespiti yuksekligi (m)")
    g.add_argument("--disarm-timeout", type=float, default=6.0)
    g.add_argument("--land-h", type=float, default=1.0, help="--land-mode: LAND'e gecis yuksekligi (m)")

    return ap.parse_args(argv)
