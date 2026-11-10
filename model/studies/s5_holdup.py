"""S5: hold-up energy. What capacitor keeps the recorder through an EN 50155 interruption and
finishes the record after a supply cut, with an FRAM ring or with an SD card only?"""

from __future__ import annotations

import numpy as np
from common import plt, save, write_json
from odo import data, holdup


def run():
    out = holdup.summary()
    opts = holdup.storage_options()
    t = np.linspace(0.0, 0.05, 101)
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    cols = ["#2a6f97", "#9a3b3b"]
    for st, c in zip(opts, cols):
        e = np.array([holdup.energy(x, st) for x in t])
        ax.plot(
            t * 1e3,
            holdup.cap_input_side(e) * 1e6,
            color=c,
            lw=1.6,
            label="%s (finish %.3g s)" % (st.name, st.t_finish),
        )
    for cls, x in (("S2", data.T_INTERRUPT_S2), ("S3", data.T_INTERRUPT_S3)):
        ax.axvline(x * 1e3, color="k", ls=":", lw=0.8)
        ax.text(x * 1e3 + 0.5, 30, "EN 50155 " + cls, fontsize=7.5)
    ax.set_yscale("log")
    ax.set_xlabel("interruption to ride through at full function (ms)")
    ax.set_ylabel("input capacitor, 16.8 V -> 9 V (uF)")
    ax.set_title("Hold-up capacitor: FRAM ring against an SD-only record")
    ax.legend(fontsize=7.5)
    save(fig, "s5_holdup.png")
    write_json("s5_holdup.json", out)
    for o in out["options"]:
        print(o["name"], "S2: %.0f uF / %.2f F supercap" % (o["S2"]["C_input_uF"], o["S2"]["C_supercap_F"]))
    return out


if __name__ == "__main__":
    run()
