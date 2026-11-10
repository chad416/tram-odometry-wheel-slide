# Requirement to test traceability

Checked by `fw/tools/trace.py` (in CI): every ID of [../SRS.md](../SRS.md) is here, every unit test
(`file:function`) and integration test (ITxx in `fw/tests/robot/odo.robot`) exists, and every Renode
tag names a known requirement.

| ID | Requirement (short) | Unit tests | Integration (Renode) | Other evidence |
|---|---|---|---|---|
| OR-01 | boot record, INIT to STAND | test_unit.c:unit_boot_and_records | IT01 | |
| OR-02 | STAND / RUN by wheel rest | test_unit.c:unit_boot_and_records | | |
| OR-05 | worst tick below 1 ms | | IT08 | |
| OR-10 | M/T wheel speed | test_basic.c:wheel_mt_speed | | |
| OR-11 | bound without edge, rest after 0.3 s | test_basic.c:wheel_decay_and_rest | | |
| OR-12 | 3-state filter, accelerometer and gyro | test_basic.c:test_kf, test_est.c:est_standstill_and_departure | IT04 | model/odo/estimator.py |
| OR-13 | standstill updates | test_est.c:est_standstill_and_departure | | |
| OR-14 | wheel gate, trust time | test_est.c:est_gate | IT03 | |
| OR-15 | re-acquisition | test_est.c:est_reacquisition | | |
| OR-16 | slide flag | test_unit.c:unit_record_catch_up | | |
| OR-17 | stuck wheel sensor | test_est.c:est_wheel_stuck | | |
| OR-20 | brake onset from the captured edge | test_unit.c:unit_brake_event, test_unit.c:unit_brake_edge_cases, test_basic.c:brake_branches | IT02 | |
| OR-21 | inertial integration in the event | test_basic.c:brake_constant_offset_is_removed, test_unit.c:unit_brake_event | IT03 | |
| OR-22 | standstill detection | test_basic.c:brake_branches, test_unit.c:unit_brake_edge_cases | IT03 | |
| OR-23 | boundary correction | test_basic.c:brake_constant_offset_is_removed, test_unit.c:unit_brake_event | IT03, IT04 | |
| OR-24 | no wheel speed in the brake distance | test_unit.c:unit_brake_event, test_basic.c:brake_constant_offset_is_removed | IT02 | model/studies/s3_montecarlo.py |
| OR-25 | brake records | test_unit.c:unit_brake_event | IT02 | |
| OR-30 | RUN record per 0.25 m | test_unit.c:unit_boot_and_records, test_unit.c:unit_record_catch_up | | |
| OR-31 | record format, CRC | test_journal.c:journal_empty_and_append | | |
| OR-32 | ring, recovery | test_journal.c:journal_wrap | | |
| OR-33 | torn record ignored | test_journal.c:journal_torn_write | IT06 | |
| OR-35 | power-fail and restore records | test_unit.c:unit_power | IT05, IT06 | |
| OR-36 | full function through S2 | | IT05 | model/odo/holdup.py |
| OR-40 | calibration command | test_unit.c:unit_calibration | IT07 | |
| OR-41 | diameter from two marks | test_basic.c:test_cal, test_unit.c:unit_calibration | IT07 | |
| OR-42 | default diameter, flag | test_unit.c:unit_calibration | | |
| OR-50 | IMU plausibility | test_est.c:est_imu_fault | | |
| OR-53 | wheels carry the brake on IMU fault | test_unit.c:unit_brake_imu_fault | | |
| OR-60 | frame decoding, CRC | test_basic.c:test_crc, test_journal.c:frame_sensor_roundtrip, test_journal.c:frame_sync_and_commands | | |
| OR-61 | bad frames rejected | test_journal.c:frame_sensor_roundtrip, test_journal.c:frame_sync_and_commands | IT08 | |
| OR-62 | status frame | test_journal.c:frame_status | IT01 | |
| OR-70 | MISRA C:2012 | | | fw/scripts/misra.sh, fw/docs/misra.md |
| OR-80 | recorded scenarios | test_scenario.c:test_scenario | IT02, IT03, IT04 | tools/export_scenarios.py |
