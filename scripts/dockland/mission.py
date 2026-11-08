"""Inis faz makinesi (saf mantik, MAVLink'ten bagimsiz).

HIZALA -> ALCAL -> SON INIS -> TEMAS -> DISARM dogrula   (veya --land-mode: ... -> LAND -> DISARM dogrula)
h = marker duzlemine yukseklik.
"""
from vehicle import LANDED_IN_AIR, LANDED_ON_GROUND

PHASE_TR = {"IDLE": "BEKLEMEDE", "ALIGN": "HIZALA", "DESCEND": "ALCAL", "FINAL": "SON INIS",
            "TOUCH": "TEMAS", "LAND": "LAND MODU", "DONE": "BITTI"}


class LandingSequencer:
    IDLE, ALIGN, DESCEND, FINAL, TOUCH, LAND, DONE = (
        "IDLE", "ALIGN", "DESCEND", "FINAL", "TOUCH", "LAND", "DONE")

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.phase = self.IDLE
        self.final = False          # son inis kilidi: marker kaybolsa da KF ile devam
        self.airborne = False       # havalanma gorulmeden temas/disarm tetiklenmez
        self.t_still = None
        self.t_touch = None
        self.t_disarm = -1e9
        self.tries = 0
        self.failed = False
        self.msg = ""

    def _finish(self, msg):
        self.phase, self.msg, self.failed = self.DONE, msg, False

    def update(self, now, running, armed, landed, h, vd, err, tracking):
        """-> {'disarm': None|'normal'|'force', 'land': bool}"""
        c = self.cfg
        out = {"disarm": None, "land": False}
        if self.phase == self.DONE:
            return out
        if not running:
            if self.phase != self.IDLE:
                self.reset()
            return out

        if armed is True and (landed == LANDED_IN_AIR or h > 0.5 or vd > 0.1):
            self.airborne = True

        if self.phase in (self.TOUCH, self.LAND):
            return self._post_step(now, armed, h, vd, out)

        if self.final and err is not None and err > 3 * c.tol:
            self.final = False                      # hata buyudu: son inisi iptal, tekrar hizala

        if c.land_mode:
            if not c.no_descend and tracking and err is not None and h < c.land_h and err < c.tol:
                self.phase, self.msg = self.LAND, "LAND moduna gecildi"
                out["land"] = True
                return out
        elif not c.no_descend and tracking and err is not None and err < 2 * c.tol and h < c.final_h:
            self.final = True

        if tracking and err is not None:
            if c.no_descend or (err >= 2 * c.tol and not self.final):
                self.phase = self.ALIGN
            else:
                self.phase = self.FINAL if self.final else self.DESCEND
        else:
            self.phase = self.FINAL if self.final else self.ALIGN

        if (not c.land_mode and not c.no_descend and self.airborne and armed is True
                and self.phase in (self.DESCEND, self.FINAL)):
            still = False
            if self.phase == self.FINAL and h < c.touch_h and abs(vd) < 0.05:
                if self.t_still is None:
                    self.t_still = now
                still = (now - self.t_still) > 1.5
            else:
                self.t_still = None
            if landed == LANDED_ON_GROUND or still:
                self.t_touch, self.t_disarm = now, -1e9
                if c.no_disarm:
                    self._finish("YERDE (disarm kapali)")
                else:
                    self.phase, self.msg = self.TOUCH, "TEMAS: disarm gonderiliyor"
                    return self._post_step(now, armed, h, vd, out)
        return out

    def _post_step(self, now, armed, h, vd, out):
        c = self.cfg
        if armed is False:
            self._finish("DISARM DOGRULANDI (heartbeat: motorlar kapali)")
            return out
        if self.phase == self.LAND:
            self.msg = "LAND modu: ArduPilot'un otomatik disarm'i bekleniyor"
            return out
        elapsed = now - self.t_touch
        if now - self.t_disarm >= 1.0:
            safe = h < 1.0 and abs(vd) < 0.3
            force = c.force_disarm and elapsed > c.disarm_timeout and safe
            out["disarm"] = "force" if force else "normal"
            self.t_disarm = now
            self.tries += 1
        if elapsed > c.disarm_timeout and not c.force_disarm:
            self.failed = True
            self.msg = "DISARM KABUL EDILMEDI: manuel disarm gerekli"
        else:
            self.msg = f"DISARM gonderildi (deneme {self.tries}), dogrulama bekleniyor"
        return out
