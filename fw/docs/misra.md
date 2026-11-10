# MISRA C:2012 compliance (odometry library)

**Scope:** `fw/lib/odo/src` and `fw/lib/odo/include`, the code shared by the host tests and the firmware.

**Checker:** cppcheck 2.13 with the MISRA addon, run by `fw/scripts/misra.sh` (CI job `host`).
- The script fails on **any** output, because cppcheck's exit code does not cover every addon finding.
- The Zephyr glue in `fw/app/src/main.c` is out of scope.

**Note:** this is a developer-level check, not a qualified MISRA compliance tool. Without the licensed
rule texts, cppcheck reports rule numbers only.

## Result

No findings except deviation D1. Fixed while setting up the check (11 Nov 2026):

| Rule | Where | Fix |
|---|---|---|
| 10.8 (composite expression cast to a different essential type) | `-(int32_t)(~raw) - 1` in `frame.c` | the complement is taken into a `uint32_t` first, then cast |
| 21.15 (memcpy between incompatible pointer types) | configuration CRC over the `float` diameters in `unit.c` | the CRC is computed over the diameters in micrometres, packed byte by byte |
| 12.1 (operator precedence) | M/T speed bound in `wheel.c` | parenthesised |
| 5.9 (internal identifiers unique) | `put16` in `journal.c` and `frame.c` | renamed `le_put16` / `be_put16` |
| 8.7 (external object used in one unit) | `odo_entry_pack`, `odo_cfg_valid` | made `static` |
| cppcheck variableScope, unreadVariable | `rise` in `unit.c` | declared where it is computed |

## Deviations

| ID | Rule | Category | Where | Reason |
|---|---|---|---|---|
| D1 | 2.5: a project should not contain unused macro definitions | Advisory | `odo.h` | `odo.h` is the public interface and the frame protocol. Its command codes and some limits are used by the application, the tests and the rig, all outside the analysed file set. |

## Design rules followed (not only checked)

- No dynamic memory, no recursion, every loop bounded.
- No library calls in the library apart from `memset` (state reset).
- All state is in caller-owned structs; the FRAM is reached only through the `odo_nvm_t` callbacks.
- Single-precision float with `f` suffixes (`-Wdouble-promotion` and `-Wfloat-equal` are on).
- Explicit casts at every narrowing; `if`/`else if` chains end with `else`.
