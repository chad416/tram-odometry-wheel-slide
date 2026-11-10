# Recorder software requirements (SRS)

Rev A, 11 Nov 2026. Derived from the system requirements in [DOSSIER.md §5](../DOSSIER.md) (SYS-x).
Statement form after Wasson 2016 ch. 22: subject, shall, outcome, condition. `fw/tools/trace.py` checks
that every requirement below appears in [docs/traceability.md](docs/traceability.md) with at least one
existing test. The parameter values are the constants in `fw/lib/odo/include/odo/odo.h`; the reason for
each value is in DOSSIER §7–§9.

## 1. Operation

- **OR-01 (SYS-1):** On the first sensor sample the recorder shall initialise the estimator, enter
  STAND and write a BOOT record.
- **OR-02 (SYS-1):** Outside an emergency brake and a calibration the recorder shall be in STAND while
  every healthy wheel is at rest, and in RUN otherwise.
- **OR-05 (SYS-6):** The worst-case processing time of one sample shall be below 1 ms (10 % of the
  10 ms sample period).

## 2. Speed and distance

- **OR-10 (SYS-2):** The recorder shall compute each wheel speed by the M/T method: the edge-count
  difference times the edge length, divided by the time between the two captured edges.
  - Edge length = pi D / 400 (100 teeth, quadrature).
- **OR-11 (SYS-2):** Without a new edge the wheel speed shall be bounded by one edge length over the
  time since the last edge, and shall be zero after 0.3 s without an edge.
- **OR-12 (SYS-2, SYS-3):** The recorder shall estimate speed, accelerometer offset and gyro offset
  with a Kalman filter predicted by the longitudinal accelerometer and the pitch-rate gyro.
  - The offset carries gravity on the tilted car body; the gyro integrates its change on vertical
    curves and under body pitch (S3, DOSSIER §9.3).
- **OR-13 (SYS-2):** While every healthy wheel is at rest and |f - c| < 0.15 m/s^2, outside a brake
  event, the recorder shall update the speed to zero, the offset to the measured specific force, and
  the gyro offset to the measured rate.
- **OR-14 (SYS-3):** A wheel speed shall be used only while it has stayed inside the gate for 0.5 s.
  - Gate: |v_wheel - v| < 0.05 m/s + 0.5 % of v, and above 1 m/s the wheel acceleration within
    0.5 m/s^2 of the inertial one.
  - A wheel at rest gives no speed measurement (OR-13 does).
- **OR-15 (SYS-3):** After 5 s without an accepted wheel, with both wheels agreeing within the gate,
  no brake input and no brake event, the recorder shall take their mean speed (re-acquisition).
- **OR-16 (SYS-4):** The records shall carry a slide flag while the vehicle moves and a wheel is gated out.
- **OR-17 (SYS-7):** A wheel sensor that shows rest for 2 s while the other wheel and the estimate move
  faster than 1 m/s, outside a brake event, shall be declared failed and no longer used.

## 3. Emergency brake

- **OR-20 (SYS-3):** A rising edge of the brake-demand input outside a brake event and a calibration
  shall start a brake event. The onset is the captured edge time: only the part of the onset sample
  after the edge is integrated.
- **OR-21 (SYS-3):** During the event the filter shall run without wheel and standstill updates, and the
  distance shall be the integral of its speed.
- **OR-22 (SYS-3):** The standstill shall be the first sample of a run of 0.3 s with |f - c| < 0.3 m/s^2.
  It shall be confirmed when every healthy wheel is at rest and the inertial speed is below 0.5 m/s, or
  after 1 s of such a run.
  - Without any healthy wheel sensor there is no standstill evidence and the event stays open.
- **OR-23 (SYS-3):** With a slide during the event, the result shall be D(T) - r T / 2 (r: inertial speed
  at the standstill, T: onset to standstill), and the offset shall be increased by r / T.
- **OR-24 (SYS-3):** The stopping distance shall not use the wheel speeds during the event, also when no
  wheel slides. The wheels enter only through the speed at the onset and the standstill.
  - Reason: the braking creep alone (0.19 % dry, 2.0 % wet at the wheel-brake demand, S1) biases the
    wheels; using them when no slide is seen (E3h) is worse in every condition (S3, decision D3).
- **OR-25 (SYS-4):** The recorder shall write a BRAKE_ON record at the onset and a BRAKE_RES record at the
  standstill, with the distance in cm, the speed at the onset, and a flag when the correction was used.

## 4. Records

- **OR-30 (SYS-4):** The recorder shall write a RUN record for every 0.25 m of travel, at most 4 per sample.
- **OR-31 (SYS-4):** Each record shall be 16 bytes: sequence number, type, flags, distance (cm), speed
  (cm/s), time (0.1 s), a type-specific field, and a CRC-16 over the first 14 bytes.
- **OR-32 (SYS-4):** The records shall form a ring of 8192 slots in FRAM. At start-up the newest record
  shall be found by serial-number arithmetic on the 16-bit sequence.
- **OR-33 (SYS-5):** A record cut by a supply loss shall be recognised by its CRC and ignored; every
  complete record before it shall survive.
- **OR-35 (SYS-5):** On the falling edge of the power-fail input the recorder shall write a PFAIL record at
  once, and a PRESTORE record when the supply returns.
- **OR-36 (SYS-5):** The recorder shall keep full function through a supply interruption of class S2 (10 ms).

## 5. Calibration

- **OR-40 (SYS-2):** The calibration command shall be accepted only in STAND, for a surveyed length of
  10 m to 2 km.
- **OR-41 (SYS-2):** The recorder shall compute each diameter from the edge counts at two rising edges of
  the mark input, D = L 400 / (pi dn), and keep it only inside 0.600–0.700 m.
  - The run is driven at walking pace (1 m/s): the mark input is sampled every 10 ms, so the mark
    position is known to 1 cm, 0.01 % of 100 m.
- **OR-42 (SYS-2):** Without a valid stored calibration the recorder shall use 0.655 m and flag every record.

## 6. Faults

- **OR-50 (SYS-7):** A specific force above 2 g or a pitch rate above 2 rad/s shall latch an IMU fault;
  the filter then holds the speed between wheel updates.
- **OR-53 (SYS-7):** With an IMU fault the wheels shall carry the brake measurement (no correction).

## 7. Interfaces

- **OR-60 (SYS-8):** The recorder shall decode sensor and command frames `A5 5A len type payload CRC16`
  (CRC-16/CCITT-FALSE over len, type and payload).
- **OR-61 (SYS-8):** A frame with a CRC error, an unknown type or a wrong length shall be rejected and
  counted, and the receiver shall resynchronise on the next sync pair.
- **OR-62 (SYS-8):** The recorder shall send a status frame every 10 samples.

## 8. Process

- **OR-70 (SYS-9):** The library shall have no MISRA C:2012 finding outside the documented deviations.
- **OR-80 (SYS-3):** On the recorded scenarios the firmware result shall be within the error budget of
  the truth and within 3 cm of the Python reference estimator.
