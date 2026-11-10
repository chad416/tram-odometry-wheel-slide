"""Sourced inputs of the study. Every value carries its reference key [Rn] (DOSSIER §17,
docs/sources.md) and a status:

    V  read in the source
    G  read off a published graph
    D  derived from V/G values by a stated method
    A  an assumption of this study

Units are SI unless the name says otherwise.
"""

from __future__ import annotations

import math

G0 = 9.80665  # standard gravity, m/s^2

# ---- Requirement-side figures -------------------------------------------------------------

# A published tram speed/event recorder specification [R19] (V): the last 1500 m are kept "with
# 0.25 m accuracy", the brake test measures "with 5 cm accuracy", the unit keeps recording
# when the vehicle supply is cut, supply 24 V nominal (16.8-36 V), -25..+85 degC.
REC_WINDOW_M = 1500.0  # V [R19]
REC_STEP_M = 0.25  # V [R19]
BRAKE_TEST_TOL_M = 0.05  # V [R19]
T_OPER_MIN_C, T_OPER_MAX_C = -25.0, 85.0  # V [R19]

# BOStrab Anlage 2, Table 2 [R1] (V): minimum mean deceleration and maximum stopping distance in
# emergency braking ("Gefahrbremsung"), empty vehicle, straight, level, dry track.
BOSTRAB_EMERGENCY = {  # v0 km/h: (a_min m/s^2, s_max m)
    20: (1.71, 9.0),
    30: (2.04, 17.0),
    40: (2.29, 27.0),
    50: (2.47, 39.0),
    60: (2.57, 54.0),
    70: (2.73, 69.0),
}

# ---- Wheel-rail contact: Polach 2005 [R2] -------------------------------------------------
# Table 2, typical parameters of real wheel-rail contact (V).
POLACH = {
    "dry": {"kA": 1.00, "kS": 0.40, "mu0": 0.55, "A": 0.40, "B": 0.60},
    "wet": {"kA": 0.30, "kS": 0.10, "mu0": 0.30, "A": 0.40, "B": 0.20},
}
# Leaf layers [R3] (V): wet leaf layer CoT 0.04 in field testing, dry leaf layers 0.06-0.11.
MU_WET_LEAF = 0.04  # V [R3]
MU_DRY_LEAF = 0.08  # D [R3]: middle of 0.06-0.11
# The leaf conditions reuse the wet Polach shape (kA, kS, A, B) with mu0 scaled so that the
# peak of the creep-force curve equals the field CoT (D, method in adhesion.py).

# Hertz contact and Kalker's coefficient (to set the initial slope of the creep curve).
E_STEEL = 210e9  # A, textbook
NU_STEEL = 0.28  # A, textbook
G_STEEL = E_STEEL / (2.0 * (1.0 + NU_STEEL))  # D
C11 = 4.12  # A: Kalker's c11 for a/b = 1 (circular contact); the slope only sets micro-creep
R_RAIL_CROWN = 0.30  # A: rail-head crown radius, m

# ---- Vehicle ----------------------------------------------------------------------------
# A 30 m, three-car, 100 % low-floor tram (A). All six axles are braked: modern low-floor
# trams brake every bogie, so no free-rolling axle is available as a speed reference (A).
M_EMPTY = 38_000.0  # kg, A
M_LOADED = 52_000.0  # kg, A (about 200 passengers at 70 kg)
N_AXLES = 6  # A
ROT_MASS_FACTOR = 0.08  # A: rotating-mass allowance on the vehicle mass
J_AXLE = 60.0  # kg m^2, A: wheelset + reflected motor and gear inertia per axle
D_NEW = 0.680  # m, V [R11] (UIC 510-2: nominal 680 mm wheel, minimum 630 mm)
D_MIN = 0.630  # m, V [R11]
DAVIS = (1500.0, 15.0, 3.5)  # N, N/(m/s), N/(m/s)^2: running resistance, A

# Magnetic track brakes: 4 magnets (A). Braking force per magnet: A, 9 kN (attractive force
# about 80 kN, sliding friction about 0.11; adhesion-independent).
MTB_N_MAGNETS = 4  # A
MTB_FORCE_N = 9_000.0  # per magnet, A
# Emergency demand on the wheel brakes (electrodynamic + friction), as a deceleration of the
# vehicle that the wheel brakes must add to the track brakes. With the track brakes this
# reproduces the BOStrab 70 km/h value of 2.73 m/s^2 on dry rail with margin (D, S1).
A_WHEEL_BRAKE_DEMAND = 2.2  # m/s^2, A
JERK_BRAKE = 3.0  # m/s^3, A: build-up of the wheel-brake torque
T_MTB_BUILD = 0.3  # s, A: magnet force build-up time

# Wheel slide protection (concept of EN 15595 [R8]; thresholds are A).
WSP_RELEASE_SLIP = 0.15  # A: release above 15 % slip
WSP_REAPPLY_SLIP = 0.05  # A: reapply below 5 %
WSP_RELEASE_DECEL = 4.0  # m/s^2 of wheel peripheral deceleration above vehicle, A
WSP_TORQUE_RATE = 8.0  # 1/s, A: fraction of full torque released/applied per second

# Gradients: modern trams run on up to about 7-8 % [R13] (V, secondary sources).
GRADE_MAX = 0.08  # V [R13]
GRADE_MAX_LEAF = 0.05  # A: envelope limit for leaf-contaminated braking (sanding not modelled)

# Body pitch under braking (secondary suspension): A, 0.15 degrees per m/s^2.
PITCH_PER_DECEL_RAD = math.radians(0.15)
PITCH_FN_HZ = 1.5  # A: body pitch natural frequency on the secondary suspension
PITCH_ZETA = 0.3  # A: its damping ratio

# ---- Sensors ------------------------------------------------------------------------------
# Wheel pulse generator: two channels, 90 degrees, 60-300 pulses/rev for traction drives
# [R12] (V, secondary). This study: 100 teeth, quadrature decoding = 400 edges/rev (A).
TEETH = 100  # A within [R12]
EDGES_PER_REV = 4 * TEETH  # D
EDGE_TIME_RES = 1e-6  # s, A: capture timer resolution

# MEMS accelerometer, ADXL355 data sheet Rev. A, Table 2 [R9] (V):
ACC_OFFSET_MAX = 75e-3 * G0  # 0 g offset max +-75 mg (typ +-25 mg), V
ACC_OFFSET_TYP = 25e-3 * G0  # V
ACC_OFFSET_TC_MAX = 0.15e-3 * G0  # +-0.15 mg/degC max, V
ACC_OFFSET_TC_TYP = 0.02e-3 * G0  # +-0.02 mg/degC typ, V
ACC_REPEAT = 3.5e-3 * G0  # repeatability +-3.5 mg (x/y), V
ACC_NOISE_DENSITY = 25e-6 * G0  # 25 ug/sqrt(Hz) at +-2 g, V
ACC_SENS_TC = 0.01e-2  # +-0.01 %/degC, V
ACC_NONLIN = 0.1e-2  # 0.1 % FS, V
ACC_BW_HZ = 50.0  # A: digital low-pass at the 100 Hz estimator rate
VIB_RMS = 0.10  # m/s^2, A: longitudinal car-body vibration seen by the accelerometer (1-10 Hz band)

# MEMS gyroscope (pitch rate), BMI088 data sheet rev. 1.9 (BST-BMI088-DS000-19) [R20] (V):
GYR_OFFSET = math.radians(1.0)  # zero-rate offset +-1 deg/s typ, V
GYR_TCO = math.radians(0.015)  # +-0.015 deg/s per K, V
GYR_NOISE_DENSITY = math.radians(0.014)  # 0.014 deg/s/sqrt(Hz), V
GYR_G_SENS = math.radians(0.1) / G0  # 0.1 deg/s per g (acceleration sensitivity), V -> rad/s per m/s^2
GYR_G_SENS_RESIDUAL = 0.1  # A: fraction left after the installation calibration of the g-sensitivity
GYR_DT_K = 2.0  # K, A: temperature change of the sensor during one run

# Doppler radar speed over ground [R10] (V): field and laboratory relative errors within
# +-0.5 % over 5-500 km/h (Du et al. 2020); stated dual-channel accuracy +-0.5 km/h < 100 km/h.
RADAR_REL_ERR = 0.005  # V [R10]
RADAR_NOISE = 0.05  # m/s rms per 10 ms sample, A

# ---- Supply and hold-up: EN 50155:2021 [R4] (V, secondary sources) ------------------------
U_NOM = 24.0  # V
U_CONT_MIN = 0.7 * U_NOM  # 16.8 V, V [R4]
U_CONT_MAX = 1.25 * U_NOM  # 30 V, V [R4]
U_FLUCT_MIN = 0.6 * U_NOM  # 14.4 V for 100 ms, V [R4]
T_INTERRUPT_S2 = 0.010  # class S2: 10 ms without loss of function, V [R4]
T_INTERRUPT_S3 = 0.020  # class S3: 20 ms, V [R4]
P_UNIT = 3.0  # W, A: recorder board without display
ETA_DCDC = 0.85  # A
U_DCDC_MIN = 9.0  # V, A: minimum input of the 24 V -> 3.3 V converter
SD_BUSY_MAX = 0.250  # s, V [R14] (SD write/erase busy limit, secondary)
SD_P_WRITE = 0.33  # W, A: 100 mA at 3.3 V while writing
FRAM_WRITE_NS_PER_BYTE = 200.0  # ns, A: 40 MHz SPI, no write delay (FRAM)
RECORD_BYTES = 16  # D: fw/lib/odo/include/odo/odo.h, odo_rec_t
