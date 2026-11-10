# Deterministic test rig for the recorder (IronPython 2.7, inside the Renode monitor).
# It replaces the sampled sensor front end and the vehicle:
#
#   - sensor frames on LPUART3, one per 10 ms of virtual time, either replayed from a
#     recorded scenario (fw/sim/scenarios/*.csv, made by tools/export_scenarios.py from the
#     vehicle and sensor models) or generated (constant speed, calibration marks)
#   - command frames (wheel diameters, calibration)
#   - power-fail comparator on GPIO2_IO00 (high = supply present)
#   - status-frame decoder
#
# Robot keywords drive it with `python "rig...."` monitor commands.

import clr

clr.AddReference("System")
from Antmicro.Renode.Time import TimeInterval  # noqa: E402
from System import Action  # noqa: E402

SYNC0, SYNC1 = 0xA5, 0x5A
T_SENSOR, T_CMD, T_STATUS = 0x10, 0x20, 0x81
CMD_DIAM0, CMD_DIAM1, CMD_CAL = 1, 2, 3


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def be32(v):
    v &= 0xFFFFFFFF
    return [(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


def frame(ftype, payload):
    body = [len(payload), ftype] + payload
    c = crc16(body)
    return [SYNC0, SYNC1] + body + [c >> 8, c & 0xFF]


def sensor_payload(t_us, c0, e0, c1, e1, acc_e5, gyro_e6, tb_us, inputs):
    return (
        be32(t_us)
        + be32(c0)
        + be32(e0)
        + be32(c1)
        + be32(e1)
        + be32(acc_e5)
        + be32(gyro_e6)
        + be32(tb_us)
        + [inputs & 0xFF]
    )


class Rig(object):
    def __init__(self, machine):
        self.m = machine
        self.uart = machine["sysbus.lpuart3"]
        self.uart.CharReceived += self._on_char
        self.gpio = machine["sysbus.gpio2"]
        self.gpio.OnGPIO(0, True)  # supply present
        self.rows = []
        self.meta = {}
        self.k = 0
        self.running = False
        self.sent = 0
        self.corrupt_every = 0
        self.gen = None
        self.buf = []
        self.need = 0
        self.status = None
        self.n_status = 0
        self.status_bad = 0
        self.pf_ms = -1.0
        self._schedule()

    # ---- time ----------------------------------------------------------------------------
    def now_ms(self):
        return self.m.ElapsedVirtualTime.TimeElapsed.TotalMilliseconds

    def _schedule(self):
        self.m.ScheduleAction(TimeInterval.FromMicroseconds(10000), Action[TimeInterval](self._tick))

    # ---- scenario replay ------------------------------------------------------------------
    def load(self, path):
        self.rows = []
        self.meta = {}
        f = open(path, "r")
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                parts = line[2:].split(" ", 1)
                if len(parts) == 2:
                    self.meta[parts[0]] = parts[1]
            elif line and line[0].isdigit():
                self.rows.append([int(x) for x in line.split(",")])
        f.close()
        self.k = 0
        return len(self.rows)

    def meta_float(self, key, i=0):
        return float(self.meta[key].split()[int(i)])

    def set_diameters_from_meta(self):
        self.command(CMD_DIAM0, int(round(self.meta_float("d_cfg_m", 0) * 1e6)))
        self.command(CMD_DIAM1, int(round(self.meta_float("d_cfg_m", 1) * 1e6)))

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def done(self):
        return self.k >= len(self.rows)

    def truth_s_mm(self):
        return self.rows[min(self.k, len(self.rows)) - 1][9]

    # ---- generated motion (calibration test) ----------------------------------------------------
    def generate(self, v_ms, d0_m, d1_m, marks_m):
        """Constant speed v from s = 0; mark input high for 0.1 m at each mark position."""
        self.gen = {
            "v": float(v_ms),
            "d": [float(d0_m), float(d1_m)],
            "marks": [float(x) for x in marks_m],
            "t": 0.0,
            "s": 0.0,
        }
        self.running = True

    def _gen_row(self):
        import math

        g = self.gen
        g["t"] += 0.01
        v = g["v"] if g["t"] > 1.0 else 0.0  # 1 s at standstill first
        g["s"] += v * 0.01
        cnt = []
        te = []
        for d in g["d"]:
            n = int(math.floor(g["s"] / (math.pi * d) * 400.0))
            cnt.append(n)
            s_edge = n * math.pi * d / 400.0
            te.append(int(round((g["t"] - (g["s"] - s_edge) / v) * 1e6)) if v > 0 else 0)
        mark = 0
        for x in g["marks"]:
            if x <= g["s"] < x + 0.1:
                mark = 2
        return [int(round(g["t"] * 1e6)), cnt[0], te[0], cnt[1], te[1], 0, 0, 0, mark]

    # ---- sending ---------------------------------------------------------------------------------
    def _tick(self, _i):
        self._schedule()
        if not self.running:
            return
        if self.gen is not None:
            r = self._gen_row()
        elif self.k < len(self.rows):
            r = self.rows[self.k]
            self.k += 1
        else:
            self.running = False
            return
        f = frame(T_SENSOR, sensor_payload(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]))
        self.sent += 1
        if self.corrupt_every and (self.sent % self.corrupt_every) == 0:
            f[9] ^= 0x10
        for b in f:
            self.uart.WriteChar(b)

    def command(self, cmd, arg):
        for b in frame(T_CMD, [int(cmd) & 0xFF] + be32(int(arg))):
            self.uart.WriteChar(b)

    def corrupt(self, every):
        self.corrupt_every = int(every)

    # ---- power-fail comparator -----------------------------------------------------------------------
    def supply(self, ok):
        self.gpio.OnGPIO(0, bool(ok))
        if not ok:
            self.pf_ms = self.now_ms()
        return self.now_ms()

    # ---- status decoder --------------------------------------------------------------------------------
    def _on_char(self, c):
        b = int(c) & 0xFF
        buf = self.buf
        if len(buf) == 0:
            if b == SYNC0:
                buf.append(b)
            return
        if len(buf) == 1:
            if b == SYNC1:
                buf.append(b)
            elif b != SYNC0:
                del buf[:]
            return
        buf.append(b)
        if len(buf) == 3:
            self.need = 4 + b + 2
        if len(buf) >= 4 and len(buf) == self.need:
            body = buf[2:-2]
            if crc16(body) == ((buf[-2] << 8) | buf[-1]) and buf[3] == T_STATUS:
                p = buf[4:-2]

                def u32(i):
                    return (p[i] << 24) | (p[i + 1] << 16) | (p[i + 2] << 8) | p[i + 3]

                def u16(i):
                    return (p[i] << 8) | p[i + 1]

                self.status = {
                    "state": p[0],
                    "flags": p[1],
                    "v_cms": u16(2),
                    "s_mm": u32(4),
                    "brake_mm": u32(8),
                    "n_brake": u16(12),
                    "v0_cms": u16(14),
                    "seq": u16(16),
                    "head": u16(18),
                    "d0_um": u32(20),
                    "d1_um": u32(24),
                }
                self.n_status += 1
            else:
                self.status_bad += 1
            del buf[:]

    def st(self, key):
        if self.status is None:
            return -1
        return self.status[key]

    # ---- firmware symbols -------------------------------------------------------------------------------
    def _sym(self, name):
        return self.m["sysbus"].GetSymbolAddress(name)

    def read_u32(self, name):
        return int(self.m["sysbus"].ReadDoubleWord(self._sym(name)))

    def write_u32(self, name, value):
        self.m["sysbus"].WriteDoubleWord(self._sym(name), int(value) & 0xFFFFFFFF)
