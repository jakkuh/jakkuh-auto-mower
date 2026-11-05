# ArduPilot config for the mower

**Nothing here has been run on real hardware.** The script has only run in simulation (Rover SITL on a laptop) and against a mock. Treat the first power-up as an untested program driving a heavy machine with a blade. Keep the key within reach, and keep the tracks off the ground and the engine off until step 8.

| File | What it is |
|---|---|
| `mower.param` | Parameters, with unknowns left as commented `# TODO` lines |
| `scripts/mower_safety.lua` | Drive conversion, track relay R1, tilt and rangefinder stops, engine relay R2 (no commands until a flag is set) |
| `test/test_mower_safety.lua` | Logic tests against a mock: `lua ardupilot/test/test_mower_safety.lua` |
| `test/sitl_check.py` | Runs the real script on Rover SITL: `python3 ardupilot/test/sitl_check.py <path>/build/sitl/bin/ardurover` |

## Load

Firmware: Rover 4.6 or later. Tested in SITL on 4.6.3 and 4.7.2.

1. Mission Planner: **Config > Full Parameter List > Load from file**, choose `mower.param`, then **Write Params**.
2. Reboot, then load and write the file again. Some params only appear once their enable param is set (`RELAYn_FUNCTION`, `SCR_ENABLE`, `GPS1_MB_TYPE`), so a single pass can skip them.
3. Copy `scripts/mower_safety.lua` to `APM/scripts/` on the SD card, using MAVFTP or a card reader. Reboot.
4. Expect the message `Mower: safety script started (draft)` and no `FAULT` message.

Set the `# TODO` values once they are measured. A `FAULT` means the script found a config it can't trust. It holds neutral, keeps R1 open, and disarms on every arm attempt until you fix the config and reboot.

## What to know before testing

- **A dead script does not fully fail safe.** MAIN1/MAIN3 fall back to 1500, but relays hold their last state and the vehicle stays armed. Seen in SITL. After that, a disarm from the radio or a failsafe will not open any relay, because the script is what opens them. With R2 energized the engine and blade keep running, and only the key stops them.
- **The fallback depends on one line.** `set_output_pwm_chan_timeout` alone holds the last PWM forever after the timeout. `send_outputs()` calls `set_output_scaled(94/95, 0)` so the channels fall back to `SERVOn_TRIM`. Don't remove it, and don't move the outputs to `SERVOn_FUNCTION` 0: there it either holds the last PWM or drops to `SERVOn_MIN`, which is full reverse on this box.
- **Before the script first runs,** MAIN1/MAIN3 should send no pulses (source reading only, bench step 2 checks it). The box is reported to stop when pulses vanish.
- **Frozen flight controller:** the source shows the IO processor outputting 0 after 200 ms without FMU data, on channels with no failsafe PWM set. That should stop the pulses, but it is not tested.
- **Motor test** arms the vehicle, so R1 closes, and it turns off the RC, GCS, and crash failsafes until the test ends. The script holds neutral for the boot delay first, so use a test longer than `BOOT_DELAY_MS`. The disarm at the end counts as a disarm after arming, so once R2 is enabled it kills the engine.
- **R2 latch:** once `R2_WIRING_CONFIRMED` is true, every disarm after arming kills the engine until the key is cycled, including a routine one. A rangefinder false trip on grass does too. The latch lives in script memory, so a script restart clears it. Restarting does not start the engine, but it re-energizes R2 without a key cycle.
- **Crash check** is active in Auto. With tracks off the ground it sees throttle with no movement, then goes to Hold and disarms after about 2 s. That is expected on the bench.
- **Rangefinder in Manual** only trips on forward command, so you can reverse away from an obstacle. Other modes except Hold trip regardless. This exception is not in the design doc. It avoids trapping the mower against an obstacle.
- **Kill switch:** `RCx_OPTION` 153 acts on switch movement. With the switch already low, the GCS can still arm. Don't treat it as a latched interlock.

## Bench-test order

Tracks off the ground, engine off, key-off stop tested first. Stop at the first surprise.

1. **Pixhawk alone, on USB, nothing else connected.** Load as above. Check the start message and no `FAULT`. Confirm no `exceeded time limit` message appears over several minutes. If you can, measure the real loop rate, since the 50 Hz target has only been measured in SITL.
2. **Meter or scope on the outputs, nothing else connected.** With the script removed, check MAIN1/MAIN3 send no pulses. With it in place, check 1500 us when disarmed. The relay board is active-low, so high means relay off. Check AUX1 and AUX2 through a full power cycle, and note any time they go low, because a low pin energizes a relay.
3. **Arm with a meter on AUX1.** AUX1 goes low on arm (relay on). MAIN1/MAIN3 hold 1500 for the boot delay, then follow steering and throttle. On disarm they go to 1500 at once and AUX1 goes high (relay off) about 0.3 s later.
4. **Relay board on, coil supply on, main relay coil not wired yet.** Same as step 3, listening for R1. Listen for clicks at power-up. Check R2 never clicks while `R2_WIRING_CONFIRMED` is false.
5. **Connect the box, meter on its outputs.** Confirm neutral reads zero. Measure which direction is positive and how the box scales, then set `FORE_AFT_*` and `DIFF_*`.
6. **Script failure test, tracks turning slowly.** Send `MAV_CMD_SCRIPTING` (42701) with param1 = 2 (`SCRIPTING_CMD_STOP`). The tracks must stop within about 0.3 s. If they don't, stop testing and rethink the design. Afterward R1 stays closed and the vehicle stays armed, as described above. Reboot to recover.
7. **Stops, one at a time, tracks turning.** Measure the real controller boot time and set `BOOT_DELAY_MS`. Then test the radio kill switch, transmitter off (SBUS-A must signal the loss), GCS link loss, the key, and tilt by lifting one side or lowering `TILT_LIMIT_DEG` for the test.
8. **Only then:** manual driving on the ground, Auto with the blade off, and the attended blade run from the bring-up order in `docs/wiring-and-code-map.md`.

Before setting `R2_WIRING_CONFIRMED`, confirm the R2 wiring and relay rating against the doc. Test first with the kill tap disconnected and a meter on the contacts: R2 energizes after boot, stays on while armed, and latches off on disarm, tilt, and script error, until reboot. Then repeat with the tap connected, clutch disengaged, and check each trigger really stops the engine. Do step 6 with the engine running and clutch disengaged, to see the engine keep running after the script dies.

## What has been tested

| Check | How | Result |
|---|---|---|
| Lua syntax | `luac -p` (5.4.8) | Passes. ArduPilot embeds 5.3, which the script's syntax also suits, and SITL runs it there |
| Script logic | 38 mock tests. 18 deliberate breaks to the script were each caught | Passes |
| Real bindings, conversion, relay library, fallback, tilt, rangefinder, Lua errors, radio-loss failsafe, R2 policy, stop from Auto | `sitl_check.py` on 4.6.3 and 4.7.2 | 56 checks per run, 3 runs per version. 5 of 6 runs were clean. See below |
| Param names and values | Read back from SITL, except HW-only ones. Those, and all binding names, checked in source for 4.6.3, 4.7.2, and master | Passes |

**Intermittent SITL failures, cause not found.** Two checks each failed once on a timeout: `level: driving` (an earlier revision on 4.6.3) and `radio loss in Manual` (final files on 4.7.2, where the disarm wasn't seen within 6 s but R1 opened just after). Neither came back in repeats of that scenario: 14 of 14 for the first, 24 of 24 for the second, with disarm about 2.3 s after radio loss in both simulator and wall-clock time. This looks like a laptop or harness stall, but that is not proven, so time the radio-loss disarm on the bench.

**In SITL but not exercised:** the `RCx_OPTION` 153 switch, GCS-loss failsafe, and crash check.

**Not tested at all:** the Pixhawk 6C Mini, IOMCU output, voltage levels, the relay board, the kill wire tap, the KEYIUAV box, track controllers, the SBUS-A, the WTRTK-982DT (SITL used a simulated GPS and compass), a real TFmini, real loop timing, vibration, or anything electrical. SITL's relay pins also differ from the real 50-51, which were only checked against the board definition and wiki.

Relay polarity (`RELAYn_INVERTED`) was added after the SITL runs and has not been simulated. Bench steps 2-4 check it.

## TODO

Measured or undecided values. All are placeholders until then.

- [ ] Script: `FORE_AFT_GAIN`, `DIFF_GAIN`, `FORE_AFT_SIGN`, `DIFF_SIGN`, and whether the box's mix matches the formula
- [ ] Script: `BOOT_DELAY_MS`, `DISARM_OPEN_DELAY_MS` (controller boot time and disarm behavior)
- [ ] Script: `TILT_LIMIT_DEG`, `TILT_DEBOUNCE_MS` (placeholder values, tune on the real slopes)
- [ ] Script: `RF_STOP_DISTANCE_M`, `RF_DEBOUNCE_MS` (expect false trips on grass and slopes)
- [ ] Script: `R2_WIRING_CONFIRMED`, once the user confirms the R2 wiring. While false and with R2 wired, the engine stays killed
- [ ] Params: `GPS1_MB_OFS_X/Y/Z` (measure antennas)
- [ ] Params: `RCMAP_*`, `MODE_CH`, `MODEn`, `RCn_OPTION` 153 (CT-6A channel test)
- [ ] Params: `SERVO5/6_MIN/MAX` (default 1100/1900 limits speed, which suits first tests)
- [ ] Params: `FS_TIMEOUT`, `FS_GCS_TIMEOUT`, `FS_THR_VALUE` (after radio-loss and link tests)
- [ ] Params: the lidar block, if fitted. Range param names differ between 4.6 (`_MIN_CM`/`_MAX_CM`) and 4.7+ (`_MIN`/`_MAX`)
- [ ] Decide whether the key alone is enough cover for a dead script holding R2 on, before running the blade
