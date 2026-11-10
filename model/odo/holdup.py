"""Hold-up energy after a supply interruption or cut (EN 50155 [R4], storage alternatives).

The recorder must (a) keep working through an interruption of class S2 (10 ms) or S3 (20 ms)
without a reset, and (b) when the supply does not come back (collision, cable torn), finish the
record so that the last 1500 m and the brake result survive.

Energy from a capacitor discharged from u1 to u2 through a converter of efficiency eta:
    E = eta * C * (u1^2 - u2^2) / 2         ->   C = 2 E / (eta (u1^2 - u2^2))
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import data


@dataclass
class Storage:
    name: str
    t_finish: float  # s to make the record complete after the supply is gone
    p_finish: float  # W drawn while finishing


def ring_bytes() -> int:
    n = int(math.ceil(data.REC_WINDOW_M / data.REC_STEP_M))
    return n * data.RECORD_BYTES


def storage_options(sd_block_bytes: int = 32 * 1024, fs_ops: int = 2) -> list[Storage]:
    """Two ways to keep the last 1500 m (DOSSIER §8.2).

    FRAM ring: every record is written to FRAM when it is made (no write delay); at a supply
    loss only the record in progress and a power-fail record are written (2 x 16 B over SPI).
    SD only: the ring is in RAM and must be flushed; the worst case allows the SD busy limit
    for every write operation (data blocks + file-system updates).
    """
    fram_t = 2 * data.RECORD_BYTES * data.FRAM_WRITE_NS_PER_BYTE * 1e-9 + 50e-6  # + ISR and logic
    n_ops = int(math.ceil(ring_bytes() / sd_block_bytes)) + fs_ops
    sd_t = n_ops * data.SD_BUSY_MAX
    return [
        Storage("FRAM ring", fram_t, data.P_UNIT),
        Storage("SD card only (RAM ring flushed)", sd_t, data.P_UNIT + data.SD_P_WRITE),
    ]


def energy(t_ride: float, st: Storage, p_unit: float = data.P_UNIT) -> float:
    """J: full function through t_ride, then finish the record."""
    return p_unit * t_ride + st.p_finish * st.t_finish


def cap_input_side(
    e_j: float, u1: float = data.U_CONT_MIN, u2: float = data.U_DCDC_MIN, eta: float = data.ETA_DCDC
) -> float:
    """F: capacitor on the 24 V input (behind the reverse diode), from u1 down to u2."""
    return 2.0 * e_j / (eta * (u1 * u1 - u2 * u2))


def cap_supercap(e_j: float, u1: float = 5.0, u2: float = 2.5, eta: float = 0.85) -> float:
    """F: supercapacitor stack (2 x 2.7 V cells at 5 V) with a buck-boost down to 2.5 V (A)."""
    return 2.0 * e_j / (eta * (u1 * u1 - u2 * u2))


def summary() -> dict:
    out = {"ring_bytes": ring_bytes(), "options": []}
    for st in storage_options():
        row = {"name": st.name, "t_finish_s": st.t_finish}
        for cls, t in (("S2", data.T_INTERRUPT_S2), ("S3", data.T_INTERRUPT_S3)):
            e = energy(t, st)
            row[cls] = {
                "energy_J": e,
                "C_input_uF": cap_input_side(e) * 1e6,
                "C_supercap_F": cap_supercap(e),
            }
        out["options"].append(row)
    return out
