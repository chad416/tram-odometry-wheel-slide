# Source register

Every input number of the study with its source, the place in the source, and how it was verified.
Status: **V** read in the source; **V2** read in a secondary source that quotes the primary; **D** derived;
**A** assumption. The code constants are in `model/odo/data.py` with the same keys.

| Key | Source | Numbers used | Where | Status |
|---|---|---|---|---|
| R1 | BOStrab (Verordnung über den Bau und Betrieb der Straßenbahnen), Anlage 2, Tables 1 and 2, gesetze-im-internet.de/strabbo_1987/anlage_2.html | Emergency braking ("Gefahrbremsung"), minimum mean deceleration / maximum stopping distance: 20 km/h 1.71 m/s² 9 m; 30: 2.04, 17; 40: 2.29, 27; 50: 2.47, 39; 60: 2.57, 54; 70: 2.73, 69. Empty vehicle, straight, level, dry track; a = v²/(3.6²·2s) | Anlage 2, Table 2 | V (page opened 10 Nov 2026); table self-consistency checked in `model/tests/test_physics.py` |
| R2 | O. Polach, "Creep forces in simulations of traction vehicles running on adhesion limit," *Wear* 258 (2005) 992–1000, doi:10.1016/j.wear.2004.03.046 | Eq. (11) creep force with kA, kS; μ = μ0[(1−A)e^{−Bw}+A]; Table 2 typical parameters: dry kA 1.00, kS 0.40, μ0 0.55, A 0.40, B 0.60 s/m; wet 0.30, 0.10, 0.30, 0.40, 0.20 | §2.4, Table 2, p. 997 | V (author's PDF, polach.ch) |
| R3 | J. Jaffé, K. Tomlinson, R. Lewis, "Moisture: the most important factor in leaf low adhesion?," *Tribology – Materials, Surfaces & Interfaces* 20(3) (2026) 189–205, doi:10.1177/17515831261434461 | Wet leaf layers in field testing: coefficient of traction 0.04; dry leaf layers 0.06–0.11; dry baseline 0.25 | Introduction (citing field data) | V (open-access PDF, White Rose eprint 240628) |
| R4 | EN 50155:2021 *Railway applications – Rolling stock – Electronic equipment* | Supply: continuous 0.7–1.25 Un, fluctuation 0.6–1.4 Un (100 ms); interruption classes S2 = 10 ms, S3 = 20 ms with normal operation maintained | §5.1 (as quoted) | V2 (supplier summaries: Bel Fuse technical paper and declarations of conformity; Televic standards overview). One article swaps S1/S2; the majority reading is used |
| R5 | EN 50657:2017 (+A1:2023) *Rolling stock applications – Software on board rolling stock*; EN 50716:2023 | EN 50657 was derived from EN 50128; its SIL1–SIL4 requirements are equivalent to EN 50128:2011; software developed to EN 50128 counts as compliant. EN 50716:2023 replaces both | Scope and introduction (as quoted by national standards bodies) | V2 |
| R6 | EN 50126-1/-2 (RAMS), EN 50129 (safety-related electronic systems) | Process frame only; no numbers used | — | background, not read |
| R7 | IEC 62625-1:2013 *On board driving data recording system – System specification* | Scope: recorder of driving data for accident investigation, applicable to all rail vehicle types; data handling after extraction excluded | Scope (IEC webstore abstract) | V2 |
| R8 | EN 15595 *Braking – Wheel slide protection* | Concept only: WSP releases and re-applies the brake per axle to use the available adhesion; thresholds in this study are assumptions | — | background, not read |
| R9 | Analog Devices ADXL354/ADXL355 data sheet, Rev. A, Table 2 (ADXL355) | 0 g offset ±25 mg typ, ±75 mg max; offset tempco ±0.02 typ, ±0.15 max mg/°C; repeatability ±3.5 mg (x, y); noise density 25 µg/√Hz (±2 g); sensitivity tempco ±0.01 %/°C; nonlinearity 0.1 % FS | Table 2 | V (PDF read) |
| R10 | L. Du, Q. Sun, J. Bai, X. Wang, T. Xu, "Speed calibration and traceability for train-borne 24 GHz continuous-wave Doppler radar sensor," *Sensors* 20(4) (2020) 1230 | Laboratory and field relative errors within ±0.5 % over 5–500 km/h; stated dual-channel accuracy ±0.5 km/h below 100 km/h | Results | V (open access) |
| R11 | UIC leaflet 510-2 | Wheel nominal 680 mm with minimum 630 mm (680–625 for some axle types) | Table of nominal/minimum diameters | V2 (search excerpt of the leaflet); used for the wear range only |
| R12 | Rail and traction speed-sensor literature and data sheets (two-channel Hall sensors, 90° phase) | 60–300 pulses per revolution for traction drives; 90° quadrature typical | — | V2; 100 teeth chosen (A) |
| R13 | Tram gradients: list of steepest gradients on adhesion railways; a low-floor tram brochure (8 %); a 2025 tram tender (7 %) | Modern trams operate on up to about 7–8 % | — | V2; 8 % envelope (A) |
| R14 | SD card write/erase busy limit 250 ms (SD Physical Layer specification, as quoted in Linux and FreeBSD driver commits) | 250 ms per write operation | — | V2 |
| R19 | A published tram speed/event recorder product specification (name withheld under the repository naming rule) | Last 1500 m kept "with 0.25 m accuracy"; brake test "with 5 cm accuracy"; records when the supply is cut; 24 V nominal, 16.8–36 V; −25..+85 °C; wheel diameter and gear ratio entered as parameters | product page | V |
| R20 | Bosch Sensortec BMI088 data sheet, rev. 1.9 (BST-BMI088-DS000-19), gyroscope | Zero-rate offset ±1 °/s; offset change over temperature ±0.015 °/s/K; noise density 0.014 °/s/√Hz; sensitivity to acceleration 0.1 °/s/g | Gyroscope specification table | V (PDF read) |
| W | C. S. Wasson, *System Engineering Analysis, Design, and Development*, 2nd ed., Wiley 2016 | Method; chapter and page references in the dossier | — | V (book read for the author's earlier dossier; printed page = PDF page − 30) |

## Assumptions (A) that drive results

| Assumption | Value | Why this value | Effect (sensitivity) |
|---|---|---|---|
| Tram | 30 m, three cars, 38–52 t, 6 axles, all braked | typical 100 % low-floor tram | all axles braked means no free-rolling reference wheel |
| Magnetic track brakes | 4 × 9 kN | ~80 kN attraction, sliding friction ~0.11 | sets the adhesion-independent deceleration |
| Wheel-brake demand | 2.2 m/s² | with the track brakes, the dry level empty tram meets BOStrab Table 2 (S1) | creep and slide level |
| WSP thresholds | release above 15 % slip or 4 m/s² wheel deceleration; re-apply below 5 % | generic two-threshold WSP | slide pattern under leaves |
| Leaf conditions | wet Polach shape with μ0 scaled to a peak of 0.04 (wet) and 0.08 (dry leaves); patches 5–30 m over half of the braking zone; gradients limited to ±5 % with leaves | R3 values; sanding not modelled | worst slide cases |
| Body pitch | 0.15° per m/s², 1.5 Hz, damping 0.3 | secondary suspension | gravity term the gyro must follow |
| Car-body vibration | 0.1 m/s² rms, 1–10 Hz band | longitudinal ride vibration | integration noise in the brake |
| Wheel-diameter calibration | ±0.05 % (default) | measured run over a surveyed 100 m at walking pace | 1500 m window error, brake v0 |
| Gyro g-sensitivity compensated to 10 % | 0.01 °/s/g | installation calibration | see S4 |
| Unit power | 3 W; DC/DC from 16.8 V down to 9 V, 85 % | recorder board without display | hold-up capacitance |
| Failure rates | see DOSSIER §10 | field-data range for such parts | S7 numbers only |
