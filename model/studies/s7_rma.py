"""S7: reliability of the record (Wasson ch. 34). For an emergency brake at a random time, what is
the probability that the recorder has no brake record, or only a degraded one, for the one-sensor
recorder (A1) and the chosen design (A3)?

Unavailability of an item with failure rate l: Q = l * t_d, where t_d is the mean time the failure
stays in service: half the interval to the test that finds it, plus the time to repair. The failure
rates are assumptions (A) in the range of field data for such parts; DOSSIER §10 lists them.
"""

from __future__ import annotations

from common import plt, save, write_json

FIT = 1e-9  # per hour
# item: failure rate (A), how it is found, time in service until repaired (h)
ITEMS = {
    "wheel sensor (bogie, harsh)": (
        20000 * FIT,
        "on-line: no edges while the other wheel and the IMU move (OR-17)",
        24.0,
    ),
    "IMU": (500 * FIT, "on-line: range check; cross-check against the wheels (OR-50)", 24.0),
    "IMU drift (latent)": (300 * FIT, "daily standstill check of the offsets", 12.0 + 24.0),
    "FRAM": (50 * FIT, "boot: recovery and CRC; read-back", 12.0 + 24.0),
    "processor board": (3000 * FIT, "watchdog, immediate", 24.0),
    "power-fail detector / hold-up": (400 * FIT, "monthly supply-interruption test", 360.0 + 24.0),
}


def run():
    q = {k: v[0] * v[2] for k, v in ITEMS.items()}
    ws = q["wheel sensor (bogie, harsh)"]
    # A1: one wheel sensor: no brake record if the sensor, the board or the memory is down;
    #     a slide (wet or leaves, about 40 % of the brakes in the S3 mix) gives a wrong record.
    a1_none = ws + q["processor board"] + q["FRAM"] + q["power-fail detector / hold-up"]
    a1_wrong = 0.4
    # A3: no record if the board or the memory is down, or both wheel sensors (no standstill
    #     evidence, FMEA W2); degraded (wheel-only, no slide correction) if the IMU is down;
    #     degraded (one wheel) if one wheel sensor is down.
    a3_none = q["processor board"] + q["FRAM"] + q["power-fail detector / hold-up"] + ws * ws
    a3_degraded = q["IMU"] + q["IMU drift (latent)"] + 2 * ws
    out = {
        "unavailability": q,
        "A1": {"no_record": a1_none, "wrong_under_slide": a1_wrong},
        "A3": {"no_record": a3_none, "degraded": a3_degraded},
    }
    write_json("s7_rma.json", out)
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    labels = ["A1 no record", "A3 no record", "A3 degraded"]
    vals = [a1_none, a3_none, a3_degraded]
    ax.bar(labels, [v * 1e6 for v in vals], color=["#9a3b3b", "#2a6f97", "#7aa6c2"])
    ax.set_ylabel("per million emergency brakes")
    ax.set_title("Brake record missing or degraded (failure rates: assumptions)")
    for i, v in enumerate(vals):
        ax.text(i, v * 1e6, "%.0f" % (v * 1e6), ha="center", va="bottom", fontsize=8)
    save(fig, "s7_rma.png")
    print({k: round(v * 1e6, 1) for k, v in zip(labels, vals)})
    return out


if __name__ == "__main__":
    run()
