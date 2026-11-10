*** Settings ***
Documentation       Shared keywords for the recorder integration tests in Renode.
Resource            ${RENODEKEYWORDS}

*** Variables ***
# Overridden by scripts/test-sim.sh
${ELF}              /root/recorder-ws/build/tram/zephyr/zephyr.elf
${OUT}              /tmp/tram-robot
${SCEN}             ${CURDIR}/../../sim/scenarios
${SIM}              ${CURDIR}/../../sim
${ST_STAND}         1
${ST_RUN}           2
${ST_BRAKE}         3
${ST_CAL}           4
${REC_PFAIL}        4
${F_PFAIL}          1

*** Keywords ***
Boot
    Execute Command             $elf=@${ELF}
    Execute Command             include @${SIM}/odo.resc
    Execute Command             include @${SIM}/rig.py
    Execute Command             python "rig = Rig(self.Machine)"
    Create Terminal Tester      sysbus.lpuart1    defaultPauseEmulation=True
    Start Emulation
    Wait For Line On Uart       odo READY    timeout=5

Advance
    [Documentation]    Run the emulation for a fixed virtual time (seconds).
    [Arguments]    ${seconds}
    Execute Command             pause
    Execute Command             emulation RunFor "${seconds}"

Rig
    [Documentation]    Evaluate an expression on the rig (IronPython inside Renode).
    [Arguments]    ${code}
    ${out}=    Execute Command    python "print(${code})"
    ${out}=    Evaluate    $out.strip()
    RETURN    ${out}

Load Scenario
    [Arguments]    ${name}
    ${n}=    Rig    rig.load('${SCEN}/${name}.csv')
    Should Be True    int($n) > 1000
    Rig    rig.set_diameters_from_meta()
    Advance    0.01

Run Scenario To End
    [Documentation]    Replay the loaded scenario, then 0.5 s more.
    Rig    rig.start()
    FOR    ${i}    IN RANGE    100
        Advance    1.0
        ${d}=    Rig    rig.done()
        IF    $d == 'True'    BREAK
    END
    Advance    0.5

Brake Error Mm
    [Documentation]    Firmware stopping distance minus the truth (mm), and minus the Python reference.
    ${fw}=    Rig    rig.st('brake_mm')
    ${t}=    Rig    rig.meta_float('d_true_m')
    ${py}=    Rig    rig.meta_float('e3_d_m')
    ${one}=    Rig    rig.meta_float('e1_d_m')
    ${err}=    Evaluate    int($fw) - float($t) * 1000.0
    ${ref}=    Evaluate    int($fw) - float($py) * 1000.0
    ${e1}=    Evaluate    (float($one) - float($t)) * 1000.0
    Log To Console    \nstopping distance ${fw} mm, truth ${t} m: error ${err} mm (python reference ${ref} mm; one wheel sensor ${e1} mm)
    RETURN    ${err}    ${ref}
