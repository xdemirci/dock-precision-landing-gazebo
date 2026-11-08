"""Marker duzlemine yukseklik (h). Rangefinder GEREKMEZ.

pnp (varsayilan): marker gorunurken PnP'den olculur ve marker'in EKF yuksekligi (m_alt = -z_ekf - h) ogrenilir.
                  Sonra h = -z_ekf - m_alt  -> PnP gurultusu sizmaz; marker kaybolsa / kadrajdan cikinca da calisir.
                  Dock yuksekligini bilmene gerek yok (1, 3, 5, 7 m pedestallar fark etmez).
rng:              DISTANCE_SENSOR varsa onu kullanir (ayni sekilde m_alt ogrenir), yoksa pnp'ye duser.
ekf:              sabit --dock-h (eski davranis).
"""
import pose


class MarkerHeight:
    OUTLIER_M = 1.5          # bu kadar sapan olcum m_alt'i hemen degistirmez
    OUTLIER_N = 20           # ...ama bu kadar ardisik sapma olursa yeni dock kabul edilir

    def __init__(self, cfg):
        self.cfg = cfg
        self.m_alt = cfg.dock_h
        self.known = cfg.h_src == "ekf"
        self._out = 0

    def _learn(self, veh, h):
        meas = -veh.d - h
        if not self.known:
            self.m_alt, self.known = meas, True
        elif abs(meas - self.m_alt) < self.OUTLIER_M:
            self.m_alt += 0.1 * (meas - self.m_alt)
            self._out = 0
        else:
            self._out += 1
            if self._out >= self.OUTLIER_N:          # baska dock'a gecildi
                self.m_alt, self._out = meas, 0

    def update(self, veh, det, now):
        """-> (h, etiket)"""
        c = self.cfg
        live = None
        if c.h_src == "rng" and veh.rng_ok(now):
            live = veh.rng
        elif c.h_src in ("pnp", "rng") and det is not None and det.good:
            h = pose.pnp_to_level(det.tvec, veh.roll, veh.pitch)[2] * c.h_scale
            live = h if h > 0.05 else None
        if live is not None:
            self._learn(veh, live)
        h = -veh.d - self.m_alt
        if live is not None:
            return h, c.h_src if c.h_src != "ekf" else "ekf"
        return h, ("ekf" if c.h_src == "ekf" or not self.known else "ekf*")
