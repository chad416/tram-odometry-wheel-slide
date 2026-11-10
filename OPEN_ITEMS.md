# Items that only hardware, a vehicle or a laboratory can close

Everything that can be done with models, host tests and the emulator is done in this repository.
These items need equipment that the author does not have. Each names the assumption it would
replace (DOSSIER §14) and the test that would close it.

| ID | Item | Replaces | How it would be closed |
|---|---|---|---|
| H1 | Wheel-rail creep and slide of a real tram under emergency braking on wet rail and leaves | Polach parameters (R2), leaf peaks (R3), WSP thresholds (A) | instrumented brake tests: wheel pulses on two bogies + a reference (optical correlation sensor or GNSS-RTK) |
| H2 | Car-body vibration and pitch spectra at the recorder mounting point | 0.1 m/s² rms 1–10 Hz; 0.15°/(m/s²), 1.5 Hz (A) | an IMU log over a day of service; replay through `model/odo/estimator.py` |
| H3 | IMU parts on the target board: offsets, g-sensitivity after the installation calibration | data-sheet values (R9, R20) | temperature-chamber and tilt-table calibration, −25..+85 °C |
| H4 | Encoder capture (QTMR/ENC) and SPI IMU drivers on a real MIMXRT1064 board | the sensor-frame front end of the emulator | bring-up; compare the captured sample with the rig's frames |
| H5 | SPI FRAM write timing and the power-fail comparator threshold | FRAM without write delay; 16 µs measured in Renode (IT05) | scope measurement of comparator edge to the last SPI byte |
| H6 | EN 50155 supply tests (interruption S2, fluctuation 0.6 Un), temperature and vibration (EN 61373), EMC (EN 50121-3-2) | the hold-up calculation S5 | type tests in an accredited laboratory |
| H7 | Surveyed calibration track with two mark detectors | 1 cm mark position at 1 m/s (OR-41) | depot installation |
| H8 | Software assessment to EN 50716 / EN 50657 at the SIL the hazard analysis assigns | the DO-178C/EN 50128-style evidence here | an independent assessor |
