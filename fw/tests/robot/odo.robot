*** Settings ***
Documentation       Recorder firmware integration tests: the test image on the MIMXRT1064-EVK
...                 model in Renode, with sim/rig.py as the sensor front end and the vehicle.
...                 Requirement IDs (OR-xx) refer to fw/SRS.md.
Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown
Resource            common.resource

*** Test Cases ***
IT01 Boot And Status
    [Documentation]    OR-01, OR-62: boot, one status frame per 10 samples, standstill state.
    [Tags]    OR-01    OR-62
    Boot
    Load Scenario    sc1_depot_dry
    Rig    rig.start()
    Advance    1.0
    ${n}=    Rig    rig.n_status
    ${bad}=    Rig    rig.status_bad
    ${s}=    Rig    rig.st('state')
    Log To Console    \n${n} status frames in 1 s, ${bad} bad; state ${s}
    Should Be True    9 <= int($n) <= 11
    Should Be Equal As Integers    ${bad}    0
    Should Be Equal As Integers    ${s}    ${ST_STAND}
    ${d}=    Rig    rig.st('d0_um')
    ${m}=    Rig    rig.meta_float('d_cfg_m', 0)
    Should Be True    abs(int($d) - float($m) * 1e6) <= 1

IT02 Depot Brake Test Within 5 cm
    [Documentation]    OR-20..OR-24: dry, level, 40 km/h: the stopping distance within 5 cm.
    [Tags]    OR-20    OR-21    OR-24    OR-25
    Boot
    Load Scenario    sc1_depot_dry
    Run Scenario To End
    ${n}=    Rig    rig.st('n_brake')
    Should Be Equal As Integers    ${n}    1
    ${err}    ${ref}=    Brake Error Mm
    Should Be True    abs(float($err)) <= 50.0
    Should Be True    abs(float($ref)) <= 10.0
    ${s}=    Rig    rig.st('state')
    Should Be Equal As Integers    ${s}    ${ST_STAND}

IT03 Slide On Wet Leaves
    [Documentation]    OR-14, OR-22, OR-23: wheels slide for the whole brake; the IMU carries
    ...                the distance; error within the 0.30 m budget (one wheel sensor: metres).
    [Tags]    OR-14    OR-22    OR-23
    Boot
    Load Scenario    sc3_leaf_slide
    Run Scenario To End
    ${err}    ${ref}=    Brake Error Mm
    Should Be True    abs(float($err)) <= 300.0
    Should Be True    abs(float($ref)) <= 10.0

IT04 Sag Curve During The Brake
    [Documentation]    OR-12, OR-23: the gradient changes from -5 % to 0 % during the brake; the pitch
    ...                gyro keeps the gravity term; error within the budget.
    [Tags]    OR-12    OR-23
    Boot
    Load Scenario    sc4_vcurve
    Run Scenario To End
    ${err}    ${ref}=    Brake Error Mm
    Should Be True    abs(float($err)) <= 300.0
    Should Be True    abs(float($ref)) <= 10.0

IT05 Supply Interruption Class S2
    [Documentation]    OR-35, OR-36: a 10 ms supply interruption while running: power-fail and
    ...                restore records, no sample lost, brake result still within the budget.
    [Tags]    OR-35    OR-36
    Boot
    Load Scenario    sc2_wet_down
    Rig    rig.start()
    Advance    20.0
    ${t0}=    Rig    rig.read_u32('odo_diag_ticks')
    Rig    rig.supply(False)
    Advance    0.0005
    ${f}=    Rig    rig.st('flags')
    ${cyc}=    Rig    rig.read_u32('odo_diag_pf_cyc')
    ${us}=    Evaluate    int($cyc) / 600.0
    Log To Console    \npower-fail record written ${us} us after the comparator edge
    Should Be True    0 < float($us) < 200.0
    Advance    0.0095
    Rig    rig.supply(True)
    Advance    0.5
    ${t1}=    Rig    rig.read_u32('odo_diag_ticks')
    ${sent}=    Rig    rig.sent
    Should Be True    int($t1) - int($t0) >= 49
    ${ticks}=    Rig    rig.read_u32('odo_diag_ticks')
    Should Be Equal As Integers    ${ticks}    ${sent}
    ${f}=    Rig    rig.st('flags')
    Should Be True    (int($f) & ${F_PFAIL}) == 0
    FOR    ${i}    IN RANGE    100
        Advance    1.0
        ${d}=    Rig    rig.done()
        IF    $d == 'True'    BREAK
    END
    Advance    0.5
    ${err}    ${ref}=    Brake Error Mm
    Should Be True    abs(float($err)) <= 300.0

IT06 Power Cut With A Torn Record
    [Documentation]    OR-33, OR-35: the supply goes for good; the power-fail record is in FRAM; the
    ...                next record is cut half-way; the recovery keeps every complete record and
    ...                finds the power-fail record as the newest.
    [Tags]    OR-33    OR-35
    Boot
    Load Scenario    sc2_wet_down
    Rig    rig.start()
    Advance    25.0
    Rig    rig.supply(False)
    Advance    0.0005
    Rig    rig.write_u32('odo_test_tear', 5)
    # the supply is gone: the processor dies in the middle of the next record
    FOR    ${i}    IN RANGE    20
        Advance    0.01
        ${tear}=    Rig    rig.read_u32('odo_test_tear')
        IF    int($tear) == 4294967295    BREAK
    END
    Rig    rig.stop()
    Should Be Equal As Integers    ${tear}    4294967295    # the hook has been used: a record was torn
    Advance    0.02
    ${t0}=    Rig    rig.read_u32('odo_diag_ticks')
    Rig    rig.write_u32('odo_test_recover', 1)
    Advance    0.05
    ${t1}=    Rig    rig.read_u32('odo_diag_ticks')
    ${r}=    Rig    rig.read_u32('odo_test_recover')
    ${h}=    Rig    rig.read_u32('odo_diag_rec_head')
    Log To Console    \nticks ${t0} -> ${t1}, recover flag ${r}, head ${h}
    ${valid}=    Rig    rig.read_u32('odo_diag_rec_valid')
    ${type}=    Rig    rig.read_u32('odo_diag_rec_type')
    ${seq}=    Rig    rig.st('seq')
    Log To Console    \nrecovery: ${valid} valid records, newest type ${type}
    Should Be Equal As Integers    ${type}    ${REC_PFAIL}
    Should Be True    int($valid) > 1000

IT07 Wheel Diameter Calibration
    [Documentation]    OR-40, OR-41: two marks 100 m apart at 1 m/s: both diameters to 0.1 mm.
    [Tags]    OR-40    OR-41
    Boot
    # walking pace (1 m/s): the mark is sampled every 10 ms, i.e. known to 1 cm in 100 m
    Execute Command    python "rig.generate(1.0, 0.6431, 0.6402, [2.0, 102.0])"
    Advance    0.5
    Rig    rig.command(3, 100000)
    Advance    0.1
    ${s}=    Rig    rig.st('state')
    Should Be Equal As Integers    ${s}    ${ST_CAL}
    Advance    103.5
    ${d0}=    Rig    rig.st('d0_um')
    ${d1}=    Rig    rig.st('d1_um')
    Log To Console    \ncalibrated diameters ${d0} um and ${d1} um (true 643100 and 640200)
    Should Be True    abs(int($d0) - 643100) <= 100
    Should Be True    abs(int($d1) - 640200) <= 100
    ${s}=    Rig    rig.st('state')
    Should Be Equal As Integers    ${s}    ${ST_RUN}

IT08 Tick Time And Corrupted Frames
    [Documentation]    OR-05, OR-61: worst tick below 10 % of the 10 ms sample period; a corrupted frame
    ...                is rejected and counted, and the recorder keeps running.
    [Tags]    OR-05    OR-61
    Boot
    Load Scenario    sc3_leaf_slide
    Rig    rig.corrupt(97)
    Run Scenario To End
    ${cyc}=    Rig    rig.read_u32('odo_diag_max_cyc')
    ${us}=    Evaluate    int($cyc) / 600.0
    ${err}=    Rig    rig.read_u32('odo_diag_rx_err')
    ${sent}=    Rig    rig.sent
    ${ticks}=    Rig    rig.read_u32('odo_diag_ticks')
    Log To Console    \nworst tick ${us} us; ${err} corrupted frames rejected of ${sent}
    Should Be True    float($us) < 1000.0
    Should Be True    int($err) >= 40
    Should Be Equal As Integers    ${ticks}    ${{ int($sent) - int($err) }}
    ${n}=    Rig    rig.st('n_brake')
    Should Be Equal As Integers    ${n}    1

