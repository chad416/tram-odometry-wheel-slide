/*
 * Tram speed and event recorder: odometry core (pure C99, no dynamic memory).
 *
 *   wheel.c    M/T speed from quadrature edge counts and last-edge capture times
 *   kf.c       3-state Kalman filter: speed v, accelerometer offset c, gyro offset b
 *   est.c      slide/spin gate, re-acquisition, standstill updates (the estimator)
 *   brake.c    emergency-brake event: inertial integration, standstill detection,
 *              two-point boundary correction
 *   journal.c  power-fail-safe record ring in FRAM (sequence number + CRC per record)
 *   cal.c      wheel-diameter calibration over a surveyed distance between two marks
 *   unit.c     the recorder: modes, records every 0.25 m, faults, power fail
 *   frame.c    serial frames: sensor frames in, commands in, status out
 *   crc.c      CRC-16/CCITT-FALSE
 *
 * The algorithm is the E3 estimator of model/odo/estimator.py (DOSSIER §8-9); the
 * requirement IDs (OR-xx) are in fw/SRS.md.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef ODO_ODO_H
#define ODO_ODO_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Timing and geometry ------------------------------------------------------------ */
#define ODO_TS            0.01f    /* s, estimator tick */
#define ODO_TICK_US       10000U
#define ODO_EDGES_PER_REV 400U     /* 100 teeth, quadrature */
#define ODO_N_AXLES       2U
#define ODO_G             9.80665f
#define ODO_PI            3.14159265f
#define ODO_REC_STEP_MM   250U     /* one RUN record every 0.25 m (OR-30) */

/* ---- Estimator parameters (model/odo/estimator.py) ---------------------------------- */
#define ODO_STAND_US      300000U  /* no edge for this long: that wheel is at rest */
#define ODO_GATE_ABS      0.05f    /* m/s */
#define ODO_GATE_REL      0.005f
#define ODO_A_GATE        0.5f     /* m/s^2: wheel vs inertial acceleration */
#define ODO_V_AGATE       1.0f     /* m/s: acceleration check only above this */
#define ODO_TRUST_TICKS   50U      /* 0.5 s consistent before a wheel is used again */
#define ODO_REACQ_TICKS   500U     /* 5 s lost with agreeing wheels: re-acquire */
#define ODO_A_STOP        0.3f     /* m/s^2: end of the deceleration */
#define ODO_A_ZUPT        0.15f    /* m/s^2: standstill updates only while |f - c| is below this */
#define ODO_V_STOP        0.5f     /* m/s: inertial speed must be below this at the stop */
#define ODO_STAND_TICKS   30U      /* 0.3 s quiet to confirm the standstill */
#define ODO_STAND_MAX     100U     /* 1 s quiet: accept without the speed check (fallback) */
#define ODO_SIG_W_ABS     0.02f
#define ODO_SIG_W_REL     0.003f
#define ODO_SIG_ZUPT      0.002f
#define ODO_SIG_ZACC      0.02f
#define ODO_SIG_ZARU      0.002f
#define ODO_Q_V           0.10f    /* m/s^2/sqrt(s) */
#define ODO_Q_C           0.0055f  /* m/s^3/sqrt(s): gyro noise + unmodelled */
#define ODO_Q_B           2e-5f    /* rad/s/sqrt(s) */

/* ---- Plausibility limits (faults, OR-50..OR-53) ------------------------------------- */
#define ODO_ACC_MAX       19.6f    /* m/s^2 (2 g) */
#define ODO_GYRO_MAX      2.0f     /* rad/s */
#define ODO_DIAM_MIN      0.600f   /* m */
#define ODO_DIAM_MAX      0.700f
#define ODO_WSTUCK_V      1.0f     /* m/s: other wheel and estimate above this ... */
#define ODO_WSTUCK_TICKS  200U     /* ... for 2 s while this wheel shows rest: wheel fault */

/* ---- Inputs ------------------------------------------------------------------------- */
#define ODO_IN_BRAKE      0x01U    /* emergency brake demand (binary input) */
#define ODO_IN_MARK       0x02U    /* calibration track mark detector */

typedef struct {
	uint32_t count;      /* quadrature edge counter */
	uint32_t t_edge_us;  /* capture time of the last edge */
} odo_wheel_raw_t;

typedef struct {
	uint32_t t_us;                        /* sample time */
	odo_wheel_raw_t wheel[ODO_N_AXLES];
	float acc;                            /* m/s^2, specific force along the car body */
	float gyro;                           /* rad/s, pitch rate of the car body */
	uint32_t t_brake_us;                  /* capture time of the last brake-input rising edge */
	uint8_t inputs;                       /* ODO_IN_* */
} odo_sample_t;

/* ---- crc.c --------------------------------------------------------------------------- */
uint16_t odo_crc16(const uint8_t *data, uint32_t len);

/* ---- wheel.c: M/T speed ---------------------------------------------------------------- */
typedef struct {
	uint32_t n_last;
	uint32_t t_last_us;
	float edge_m;   /* pi * diameter / ODO_EDGES_PER_REV */
	float v;        /* m/s */
	float v_prev;
	bool seen;      /* an edge has been seen since init */
} odo_wheel_t;

void odo_wheel_init(odo_wheel_t *w, float diameter_m, const odo_wheel_raw_t *in, uint32_t t_us);
void odo_wheel_update(odo_wheel_t *w, const odo_wheel_raw_t *in, uint32_t t_us);
bool odo_wheel_at_rest(const odo_wheel_t *w);

/* ---- kf.c ------------------------------------------------------------------------------- */
typedef struct {
	float x[3];     /* v, c, b */
	float p[3][3];
} odo_kf_t;

void odo_kf_init(odo_kf_t *k, float c0);
void odo_kf_predict(odo_kf_t *k, float acc, float gyro);
void odo_kf_update(odo_kf_t *k, uint32_t j, float z, float sig);
void odo_kf_set_speed(odo_kf_t *k, float v, float sig);

/* ---- est.c ------------------------------------------------------------------------------- */
typedef struct {
	odo_kf_t kf;
	odo_wheel_t wheel[ODO_N_AXLES];
	uint16_t trust[ODO_N_AXLES];
	uint16_t lost;
	uint16_t stuck[ODO_N_AXLES];
	bool wheel_fault[ODO_N_AXLES];
	bool imu_fault;
	uint8_t used;   /* bit k: wheel k used this tick */
	bool rest;      /* all healthy wheels at rest */
} odo_est_t;

void odo_est_init(odo_est_t *e, const float diam[ODO_N_AXLES], const odo_sample_t *s0);
/* One tick. event: an emergency-brake event is running (no wheel updates, no standstill
 * updates); brake_in: brake demand input (blocks re-acquisition). */
void odo_est_step(odo_est_t *e, const odo_sample_t *s, bool event, bool brake_in);
float odo_est_speed(const odo_est_t *e);

/* ---- brake.c ----------------------------------------------------------------------------- */
typedef struct {
	bool active;
	bool done;       /* a result is available */
	float v0;        /* m/s at the onset */
	float v;         /* inertial speed */
	float d;         /* m, live distance since the onset */
	float t;         /* s since the captured onset */
	float w;         /* weight of the next tick (part of the onset tick after the edge) */
	float s0;        /* m, recorder distance at the onset */
	uint16_t quiet;  /* consecutive ticks with |f - c| < ODO_A_STOP */
	float q_v;       /* speed, distance and time at the start of the quiet run */
	float q_d;
	float q_t;
	float result_d;  /* m, corrected stopping distance */
	float result_r;  /* m/s, speed residual removed by the correction */
	float result_t;  /* s, onset to standstill */
} odo_brake_t;

/* frac: part of the onset tick that lies after the captured brake edge, 0..1 */
void odo_brake_start(odo_brake_t *b, float v0, float frac);
typedef struct {
	float v;          /* inertial speed (the filter without wheel updates) */
	float f_minus_c;  /* deceleration seen by the accelerometer */
	bool rest;        /* every healthy wheel at rest */
} odo_brake_in_t;

/* Returns true on the tick the standstill is confirmed (result_* valid): the corrected
 * inertial distance (OR-23). */
bool odo_brake_step(odo_brake_t *b, const odo_brake_in_t *in);

/* ---- journal.c ------------------------------------------------------------------------------ */
#define ODO_SLOTS      8192U   /* 128 KiB of FRAM: 1500 m at 0.25 m = 6000 RUN records + events */
#define ODO_SLOT_BYTES 16U

enum {
	ODO_REC_NONE = 0,
	ODO_REC_RUN = 1,
	ODO_REC_BRAKE_ON = 2,
	ODO_REC_BRAKE_RES = 3,
	ODO_REC_PFAIL = 4,
	ODO_REC_PRESTORE = 5,
	ODO_REC_BOOT = 6,
	ODO_REC_CAL = 7
};

typedef struct {
	uint16_t seq;
	uint8_t type;
	uint8_t flags;   /* ODO_F_* status at the time of the record */
	uint32_t s_cm;   /* distance, cm */
	uint16_t v_cms;  /* speed, cm/s */
	uint16_t t_ds;   /* time, 0.1 s, wrapping */
	uint16_t aux;    /* type-specific (BRAKE_RES: distance in cm; CAL: axle mask) */
	uint16_t crc;    /* CRC-16 over the first 14 bytes */
} odo_entry_t;

typedef struct {
	/* returns the number of bytes actually written (fewer on a torn write) */
	uint32_t (*write)(void *ctx, uint32_t addr, const uint8_t *src, uint32_t len);
	void (*read)(void *ctx, uint32_t addr, uint8_t *dst, uint32_t len);
	void *ctx;
} odo_nvm_t;

typedef struct {
	odo_nvm_t nvm;
	uint32_t head;      /* next slot */
	uint16_t seq;       /* next sequence number */
	uint32_t valid;     /* valid records found by the last recovery */
	uint32_t written;   /* records appended since the last recovery */
	uint32_t torn;      /* appends that did not complete */
} odo_journal_t;

bool odo_entry_unpack(const uint8_t in[ODO_SLOT_BYTES], odo_entry_t *e);
void odo_journal_recover(odo_journal_t *j, const odo_nvm_t *nvm);
bool odo_journal_append(odo_journal_t *j, odo_entry_t *e);
/* back = 0: newest record. Returns false if that slot holds no valid record. */
bool odo_journal_get(const odo_journal_t *j, uint32_t back, odo_entry_t *e);

/* ---- cal.c ------------------------------------------------------------------------------------ */
typedef enum { ODO_CAL_IDLE = 0, ODO_CAL_ARMED = 1, ODO_CAL_RUNNING = 2 } odo_cal_state_t;

typedef struct {
	odo_cal_state_t state;
	uint32_t length_mm;
	uint32_t n0[ODO_N_AXLES];
	bool prev_mark;
	float diam[ODO_N_AXLES];  /* result */
	uint8_t ok_mask;          /* bit k: axle k result inside the limits */
} odo_cal_t;

void odo_cal_start(odo_cal_t *c, uint32_t length_mm);
/* Returns true when the second mark has been passed (result in diam / ok_mask). */
bool odo_cal_step(odo_cal_t *c, const odo_sample_t *s);

/* ---- unit.c ------------------------------------------------------------------------------------ */
typedef enum {
	ODO_ST_INIT = 0,
	ODO_ST_STAND = 1,
	ODO_ST_RUN = 2,
	ODO_ST_BRAKE = 3,
	ODO_ST_CAL = 4
} odo_state_t;

#define ODO_F_PFAIL   0x01U  /* supply lost (power-fail input active) */
#define ODO_F_WHEEL0  0x02U  /* wheel sensor 0 failed */
#define ODO_F_WHEEL1  0x04U
#define ODO_F_IMU     0x08U  /* IMU failed: wheel-only odometry */
#define ODO_F_SLIDE   0x10U  /* a wheel is gated out (slide/spin) */
#define ODO_F_CORR    0x20U  /* BRAKE_RES: boundary correction applied */
#define ODO_F_CALBAD  0x40U  /* calibration rejected / default diameter in use */

typedef struct {
	float diam[ODO_N_AXLES];
	uint16_t crc;
} odo_cfg_t;

typedef struct {
	odo_state_t state;
	odo_est_t est;
	odo_brake_t brake;
	odo_journal_t jr;
	odo_cal_t cal;
	odo_cfg_t cfg;
	float s;              /* m, distance (fused) */
	uint32_t next_rec;    /* next RUN record at this distance, mm */
	bool prev_brake;
	bool pfail;
	bool started;
	uint32_t t_us;
	uint32_t ticks;
	uint8_t flags;
	/* last brake result */
	uint16_t n_brake;
	float last_d;
	float last_v0;
} odo_unit_t;

void odo_unit_init(odo_unit_t *u, const odo_nvm_t *nvm, const odo_cfg_t *cfg);
void odo_unit_tick(odo_unit_t *u, const odo_sample_t *s);
void odo_unit_power(odo_unit_t *u, bool supply_ok);
bool odo_unit_cal_start(odo_unit_t *u, uint32_t length_mm);
void odo_cfg_seal(odo_cfg_t *c);

/* ---- frame.c ----------------------------------------------------------------------------------- */
#define ODO_SYNC0        0xA5U
#define ODO_SYNC1        0x5AU
#define ODO_T_SENSOR     0x10U
#define ODO_T_CMD        0x20U
#define ODO_T_STATUS     0x81U
#define ODO_SENSOR_LEN   33U
#define ODO_CMD_LEN      5U
#define ODO_STATUS_LEN   28U
#define ODO_MAX_FRAME    (ODO_STATUS_LEN + 6U)

#define ODO_CMD_DIAM0    1U   /* arg: diameter in um */
#define ODO_CMD_DIAM1    2U
#define ODO_CMD_CAL      3U   /* arg: surveyed length in mm */

typedef struct {
	uint8_t buf[48];
	uint32_t n;
	uint32_t need;
	uint32_t crc_err;
	uint32_t ok;
} odo_rx_t;

typedef enum { ODO_RX_NONE = 0, ODO_RX_SENSOR = 1, ODO_RX_CMD = 2 } odo_rx_kind_t;

typedef struct {
	uint8_t cmd;
	uint32_t arg;
} odo_cmd_t;

void odo_rx_init(odo_rx_t *r);
/* Feed one byte. On a complete valid frame returns its kind and fills *s or *c. */
odo_rx_kind_t odo_rx_byte(odo_rx_t *r, uint8_t b, odo_sample_t *s, odo_cmd_t *c);
uint32_t odo_encode_sensor(const odo_sample_t *s, uint8_t *out, uint32_t cap);
uint32_t odo_encode_status(const odo_unit_t *u, uint8_t *out, uint32_t cap);

#endif /* ODO_ODO_H */
