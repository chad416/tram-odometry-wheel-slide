# Changelog

## Rev A, 11 Nov 2026

- Engineering dossier (Wasson spine) and PDF; source register.
- Models: Polach wheel-rail creep, per-axle tram brake dynamics with WSP, sensor models (wheel pulse
  generators, MEMS accelerometer and gyro, Doppler radar), estimators E1–E5, Monte Carlo S3, error
  budget S4, hold-up S5, analysis of alternatives S6, reliability S7.
- Firmware library `fw/lib/odo` (C99): M/T wheel speed, 3-state filter, slide gate, brake measurement
  with boundary correction, power-fail-safe FRAM ring, calibration, frames. Host tests with 100 %
  statement, branch and MC/DC coverage; MISRA C:2012 check; requirement traceability.
- Zephyr application for the MIMXRT1064-EVK; Renode integration tests IT01–IT08 against recorded
  scenarios from the models.
