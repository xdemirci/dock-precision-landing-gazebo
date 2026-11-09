"""ArduPilot ile MAVLink: durum okuma (bloklamadan) ve komutlar."""
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
MODE_NAMES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
              6: "RTL", 9: "LAND", 16: "POSHOLD"}
GUIDED = 4
LANDED_ON_GROUND = 1      # MAV_LANDED_STATE_ON_GROUND
LANDED_IN_AIR = 2         # MAV_LANDED_STATE_IN_AIR


class Vehicle:
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn)
        self.m.wait_heartbeat()
        print("heartbeat ok")
        self.n = self.e = self.d = 0.0
        self.vn = self.ve = self.vd = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.mode, self.armed, self.landed = -1, None, 0
        self.pos_ok = False
        self.rng, self.t_rng = None, 0.0
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            MAV.MAV_DATA_STREAM_ALL, 20, 1)
        self._interval(MAV.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 200000)    # landed_state 5 Hz
        self._interval(MAV.MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000)       # rangefinder 10 Hz

    def _interval(self, msg_id, us):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, us, 0, 0, 0, 0, 0)

    @property
    def guided(self):
        return self.mode == GUIDED

    @property
    def mode_name(self):
        return MODE_NAMES.get(self.mode, str(self.mode))

    def rng_ok(self, now):
        return self.rng is not None and now - self.t_rng < 1.0

    def poll(self):
        # NOT: pymavlink'in bloklayan yardimcilarini (motors_disarmed_wait vb.) kullanma: video dongusunu dondurur.
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.n, self.e, self.d = msg.x, msg.y, msg.z
                self.vn, self.ve, self.vd = msg.vx, msg.vy, msg.vz
                self.pos_ok = True
            elif t == "ATTITUDE":
                self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            elif (t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system
                  and msg.autopilot != MAV.MAV_AUTOPILOT_INVALID):
                self.mode = msg.custom_mode
                self.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "EXTENDED_SYS_STATE":
                self.landed = msg.landed_state
            elif t == "DISTANCE_SENSOR" and msg.orientation == 25:        # asagi bakan
                self.rng, self.t_rng = msg.current_distance / 100.0, time.monotonic()

    def send_vel_ned(self, vn, ve, vz=0.0, yaw_rate=None):
        mask = 0x0FC7 if yaw_rate is None else 0x07C7      # sadece hiz | hiz + yaw_rate
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            MAV.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, vn, ve, vz, 0, 0, 0, 0, yaw_rate or 0.0)

    def set_mode(self, name):
        self.m.set_mode(name)

    def send_disarm(self, force=False):
        # param2=21196: havada bile zorla. DIKKAT: motorlari aninda keser.
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 21196 if force else 0, 0, 0, 0, 0, 0)
