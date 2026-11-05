#!/usr/bin/env python3
"""Runs mower_safety.lua on ArduPilot Rover SITL and checks real output values.

Needs a built ardurover binary and pymavlink:
    python3 ardupilot/test/sitl_check.py /path/to/build/sitl/bin/ardurover

This is a simulator on a laptop, not the Pixhawk. It checks the real Lua bindings,
SRV_Channel logic, and relay library. It does not check IOMCU output, electrical
behavior, the KEYIUAV box, real timing, or anything else about real hardware.
"""
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "mower_safety.lua")
PARAM_FILE = os.path.join(HERE, "..", "mower.param")
HW_ONLY = {"BRD_SAFETY_DEFLT", "BRD_SER1_RTSCTS", "BRD_SER2_RTSCTS"}
# Named only in commented-out TODO lines of mower.param. They must exist as well.
TODO_NAMES = ["SERVO5_MIN", "SERVO5_MAX", "SERVO6_MIN", "SERVO6_MAX", "GPS1_MB_OFS_X",
              "GPS1_MB_OFS_Y", "GPS1_MB_OFS_Z", "FS_TIMEOUT", "FS_GCS_TIMEOUT", "FS_THR_VALUE",
              "RCMAP_ROLL", "RCMAP_THROTTLE", "MODE_CH", "MODE1", "RC6_OPTION",
              "SERIAL2_PROTOCOL", "SERIAL2_BAUD", "RNGFND1_TYPE", "RNGFND1_ORIENT"]

# SITL has no UM982, so the simulated GPS and compass stand in. Everything else
# comes from mower.param. Later default files override earlier ones. SITL GPIO only
# accepts pins below 16, so the relay pins move here. Scenario B checks the real ones.
SIM_OVERRIDES = """
GPS1_TYPE 1
EK3_SRC1_YAW 1
ARMING_CHECK 0
FS_THR_ENABLE 0
FS_GCS_ENABLE 0
FS_CRASH_CHECK 0
RELAY1_PIN 8
RELAY2_PIN 9
"""
BOOT_S, DISARM_S = 3.0, 0.3
results = []


def check(name, condition, detail=""):
    results.append((name, bool(condition), detail))
    print(("  ok    " if condition else "  FAIL  ") + name + (f"   [{detail}]" if detail else ""), flush=True)


class Sitl:
    def __init__(self, binary, repo, lua_text=None, extra_params="", probes=None, sim_overrides=SIM_OVERRIDES):
        self.dir = tempfile.mkdtemp(prefix="mower-sitl-")
        os.makedirs(os.path.join(self.dir, "scripts"))
        if lua_text is not None:
            with open(os.path.join(self.dir, "scripts", "mower_safety.lua"), "w") as f:
                f.write(lua_text)
        for name, text in (probes or {}).items():
            with open(os.path.join(self.dir, "scripts", name), "w") as f:
                f.write(text)
        defaults = [os.path.join(repo, "Tools/autotest/default_params/rover.parm"),
                    os.path.join(repo, "Tools/autotest/default_params/rover-skid.parm")]
        defaults.append(os.path.join(self.dir, "mower.parm"))
        shutil.copy(PARAM_FILE, defaults[-1])
        defaults.append(os.path.join(self.dir, "sim.parm"))
        with open(defaults[-1], "w") as f:
            f.write(sim_overrides + extra_params)
        self.log = open(os.path.join(self.dir, "sitl.log"), "w")
        self.proc = subprocess.Popen(
            [binary, "-S", "--model", "rover-skid", "--speedup", "1", "--wipe", "-I0",
             "--defaults", ",".join(defaults)],
            cwd=self.dir, stdout=self.log, stderr=subprocess.STDOUT)
        self.m = None
        for _ in range(40):
            try:
                self.m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255, retries=0)
                break
            except OSError:
                time.sleep(0.25)
        assert self.m is not None, "could not connect to SITL"
        self.m.wait_heartbeat(timeout=30)
        self.servo = {}
        self.relay_on = None
        self.armed = False
        self.mode = None
        self.text = []
        self.override = None
        self.rf_m = None
        self.last_tick = 0
        self.set_interval(36, 20)     # SERVO_OUTPUT_RAW
        self.set_interval(376, 20)    # RELAY_STATUS
        self.pump(2)

    def set_interval(self, msg_id, hz):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            now = time.time()
            if now - self.last_tick > 0.1:
                self.last_tick = now
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                if self.override:
                    self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component,
                                                         *self.override)
                if self.rf_m is not None:
                    self.m.mav.distance_sensor_send(0, 10, 4000, int(self.rf_m * 100), 0, 1, 0, 0)
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "SERVO_OUTPUT_RAW":
                for i in range(1, 15):
                    self.servo[i] = getattr(msg, f"servo{i}_raw", None)
            elif kind == "RELAY_STATUS":
                self.relay_on = msg.on
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = msg.custom_mode
            elif kind == "STATUSTEXT":
                self.text.append(msg.text)

    def r(self, index):
        return None if self.relay_on is None else bool(self.relay_on & (1 << index))

    def set_rc(self, steer=1500, thr=1500):
        self.override = [steer, 1500, thr, 1500, 1500, 1500, 1500, 1500]

    def command(self, cmd, p1=0, p2=0):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0,
                                     p1, p2, 0, 0, 0, 0, 0)

    def arm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)

    def disarm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def set_mode(self, number):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number)

    def wait(self, predicate, timeout):
        """Returns the elapsed time when predicate first holds, else None."""
        start = time.time()
        while time.time() - start < timeout:
            self.pump(0.02)
            if predicate():
                return time.time() - start
        return None

    def get_param(self, name):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                           name.encode(), -1)
        end = time.time() + 2
        while time.time() < end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id == name:
                return msg.param_value
        return None

    def set_param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.pump(0.3)

    def upload_rover_mission(self):
        """Uploads home plus one distant waypoint, so Auto will engage."""
        msg = None
        end = time.time() + 40
        while time.time() < end and msg is None:
            self.pump(0.2)
            msg = self.m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if msg is not None and msg.lat == 0:
                msg = None
            if msg is None:
                self.set_interval(33, 5)
        assert msg is not None, "no position"
        items = [(msg.lat, msg.lon), (msg.lat + 1000, msg.lon), (msg.lat + 2000, msg.lon)]
        self.m.mav.mission_count_send(self.m.target_system, self.m.target_component, len(items))
        sent = 0
        end = time.time() + 15
        while time.time() < end:
            req = self.m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                    blocking=True, timeout=0.5)
            if req is None:
                continue
            if req.get_type() == "MISSION_ACK":
                return req.type == 0 and sent == len(items)
            lat, lon = items[req.seq]
            self.m.mav.mission_item_int_send(self.m.target_system, self.m.target_component, req.seq,
                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
                                             lat, lon, 0)
            sent = max(sent, req.seq + 1)
        return False

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.log.close()
        try:
            self.m.close()
        except Exception:
            pass
        time.sleep(0.5)
        shutil.rmtree(self.dir, ignore_errors=True)


def main_lua(replacements=None, inject=None):
    with open(SCRIPT) as f:
        text = f.read()
    for old, new in (replacements or {}).items():
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    if inject:
        anchor = "    local armed = arming:is_armed()\n\n    if fault_msg == nil"
        assert text.count(anchor) == 1
        text = text.replace(anchor, inject + "\n" + anchor)
    return text


FALLBACK_PROBE = """
local t0 = millis():toint()
{setup}
local function loop()
    if millis():toint() - t0 > 3000 then
        gcs:send_text(6, "probe stopped sending")
        return
    end
    SRV_Channels:set_output_pwm_chan_timeout(0, 1700, 150)
    SRV_Channels:set_output_pwm_chan_timeout(2, 1700, 150)
    return loop, 15
end
return loop, 15
"""


def scenario_fallback(binary, repo):
    print("\nA. What MAIN1/MAIN3 do after set_output_pwm_chan_timeout stops being called")
    cases = [
        ("func 94, no scaled call", "94", "", 1700),
        ("func 94, scaled 0 once", "94", "SRV_Channels:set_output_scaled(94, 0)", 1500),
        ("func 0, no scaled call", "0", "", 1700),
        # Scaled 0 on function 0 has no angle setup, so it resolves to SERVOn_MIN.
        # On this box that is full reverse, so function 0 is not usable as a fallback.
        ("func 0, scaled 0 once", "0", "SRV_Channels:set_output_scaled(0, 0)", 1000),
    ]
    for label, func, setup, expected in cases:
        # MIN is pinned so the function 0 result is an explicit SERVOn_MIN.
        extra = f"SERVO1_FUNCTION {func}\nSERVO3_FUNCTION {func}\nSERVO1_MIN 1000\nSERVO3_MIN 1000\n"
        s = Sitl(binary, repo, None, extra, {"probe.lua": FALLBACK_PROBE.format(setup=setup)})
        try:
            s.wait(lambda: s.servo.get(1) == 1700, 5)
            during = s.servo.get(1), s.servo.get(3)
            s.wait(lambda: any("probe stopped" in t for t in s.text), 6)
            s.pump(1.0)
            after = s.servo.get(1), s.servo.get(3)
            check(f"{label}: after timeout MAIN1={after[0]} MAIN3={after[1]}",
                  during == (1700, 1700) and after == (expected, expected),
                  f"during={during}, expected {expected}")
        finally:
            s.close()


def scenario_params(binary, repo):
    print("\nB. Every name in mower.param exists and holds the file's value in SITL")
    wanted = {}
    with open(PARAM_FILE) as f:
        for line in f:
            match = re.match(r"^([A-Z][A-Z0-9_]+),(-?[0-9.]+)\s*$", line)
            if match:
                wanted[match.group(1)] = float(match.group(2))
    check(f"parsed {len(wanted)} active params from mower.param", len(wanted) > 30)
    s = Sitl(binary, repo, main_lua(), "", sim_overrides="GPS1_TYPE 25\n")
    try:
        bad = []
        for name, value in wanted.items():
            if name in HW_ONLY:
                continue
            got = s.get_param(name)
            if got is None or not math.isclose(got, value, abs_tol=1e-3):
                bad.append(f"{name}={got} want {value}")
        check("all active params read back with the file's value", not bad, "; ".join(bad))
        missing = [n for n in TODO_NAMES if s.get_param(n) is None]
        check("all names in TODO lines exist", not missing, ", ".join(missing))
    finally:
        s.close()


def scenario_sequence(binary, repo):
    print("\nC. Arm, drive conversion, script death, restart, disarm")
    s = Sitl(binary, repo, main_lua())
    try:
        check("script started", s.wait(lambda: any("safety script started" in t for t in s.text), 5) is not None
              or any("safety script started" in t for t in s.text))
        s.pump(1)
        check("disarmed: MAIN1/3 are 1500", (s.servo.get(1), s.servo.get(3)) == (1500, 1500),
              f"{s.servo.get(1)},{s.servo.get(3)}")
        check("disarmed: R1 off", s.r(0) is False, f"relay byte {s.relay_on}")

        s.set_rc(thr=1800)
        s.arm()
        elapsed = s.wait(lambda: s.armed and s.r(0) is True, 3)
        check("arm: R1 closes", elapsed is not None, f"{elapsed}s")
        s.pump(1.0)
        check("boot delay: MAIN5/6 already moving, MAIN1/3 held at 1500",
              s.servo.get(5, 0) > 1600 and (s.servo.get(1), s.servo.get(3)) == (1500, 1500),
              f"L={s.servo.get(5)} R={s.servo.get(6)} FA={s.servo.get(1)} D={s.servo.get(3)}")
        moved = s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 1)
        check("after boot delay: motion allowed", moved is not None, f"waited {moved}s more")

        for label, steer, thr in (("forward", 1500, 1800), ("reverse", 1500, 1200),
                                  ("pivot right", 1800, 1500), ("arc", 1700, 1700)):
            s.set_rc(steer=steer, thr=thr)
            s.pump(1.5)
            left, right = s.servo[5], s.servo[6]
            fore_aft, diff = s.servo[1], s.servo[3]
            want_fa = 1500 + ((left - 1500) + (right - 1500)) / 2
            want_d = 1500 + ((left - 1500) - (right - 1500)) / 2
            check(f"{label}: read 73/74, formula holds", abs(fore_aft - want_fa) <= 3 and abs(diff - want_d) <= 3,
                  f"L={left} R={right} FA={fore_aft} D={diff}")

        s.set_rc(thr=1800)
        s.pump(1.0)
        check("before stopping script: commanding motion", s.servo[1] > 1600, f"FA={s.servo[1]}")
        s.command(mavutil.mavlink.MAV_CMD_SCRIPTING, mavutil.mavlink.SCRIPTING_CMD_STOP)
        elapsed = s.wait(lambda: s.servo.get(1) == 1500 and s.servo.get(3) == 1500, 2)
        check("script stopped: MAIN1/3 fall back to 1500", elapsed is not None, f"{elapsed}s, L={s.servo.get(5)}")
        s.pump(1.0)
        check("script stopped: outputs stay at 1500 with motion still commanded",
              s.servo.get(1) == 1500 and s.servo.get(5) > 1600, f"FA={s.servo.get(1)} L={s.servo.get(5)}")
        check("KNOWN GAP, script stopped: R1 stays closed and vehicle stays armed",
              s.r(0) is True and s.armed, f"R1={s.r(0)} armed={s.armed}")

        s.command(mavutil.mavlink.MAV_CMD_SCRIPTING, mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART)
        s.pump(1.0)
        check("restart while armed: neutral first", s.servo.get(1) == 1500, f"FA={s.servo.get(1)}")
        moved = s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 2)
        check("restart while armed: motion after boot delay", moved is not None, f"{moved}s")

        s.disarm()
        start = time.time()
        off = s.wait(lambda: s.r(0) is False, 3)
        s.pump(0.1)
        check("normal disarm: R1 opens after the delay, not instantly",
              off is not None and 0.15 <= off <= 1.0, f"{off}s after command, delay {DISARM_S}s")
        check("normal disarm: MAIN1/3 at 1500", (s.servo.get(1), s.servo.get(3)) == (1500, 1500))
        errors = [t for t in s.text if "ERROR" in t or "FAULT" in t or "Exceeded" in t]
        check("no script errors in this run", not errors, "; ".join(errors))
    finally:
        s.close()


def scenario_rate(binary, repo):
    print("\nD. Loop rate on this laptop (does not predict the Pixhawk)")
    inject = ('    hz_n = (hz_n or 0) + 1\n'
              '    if hz_t == nil or now - hz_t >= 1000 then hz_t = now; '
              'gcs:send_named_float("HZ", hz_n); hz_n = 0 end')
    s = Sitl(binary, repo, main_lua(inject=inject))
    try:
        samples = []
        end = time.time() + 6
        while time.time() < end:
            msg = s.m.recv_match(type="NAMED_VALUE_FLOAT", blocking=True, timeout=0.5)
            if msg is not None and msg.name == "HZ":
                samples.append(msg.value)
        samples = samples[1:]
        check("loop reaches 50 Hz", samples and min(samples) >= 50, f"samples {samples}")
    finally:
        s.close()


def scenario_rangefinder(binary, repo):
    print("\nE. Rangefinder stop with a real backend (MAVLink type, forward)")
    extra = "RNGFND1_TYPE 10\nRNGFND1_ORIENT 0\nRNGFND1_MIN_CM 10\nRNGFND1_MAX_CM 600\nRNGFND1_MIN 0.1\nRNGFND1_MAX 6\n"
    s = Sitl(binary, repo, main_lua(), extra)
    try:
        s.rf_m = 5.0
        s.set_rc(thr=1800)
        s.arm()
        s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 3)
        s.pump(1.0)
        check("5 m: Manual, still driving", s.mode == 0 and s.servo[1] > 1600, f"mode={s.mode}")
        s.rf_m = 1.5
        held = s.wait(lambda: s.mode == 4, 3)
        check("1.5 m while driving forward: goes to Hold", held is not None, f"{held}s")
        s.pump(0.1)
        check("Hold: outputs neutral at once, still armed, R1 closed",
              s.servo.get(1) == 1500 and s.armed and s.r(0) is True, f"FA={s.servo.get(1)}")
        s.set_rc(thr=1200)
        s.pump(0.3)
        s.set_mode(0)
        s.wait(lambda: s.mode == 0, 2)
        s.pump(2.0)
        check("Manual reverse with obstacle ahead: no re-trip", s.mode == 0 and s.servo[1] < 1400,
              f"mode={s.mode} FA={s.servo[1]}")
        s.set_rc(thr=1800)
        held = s.wait(lambda: s.mode == 4, 3)
        check("Manual forward with obstacle ahead: trips again", held is not None, f"{held}s")
        errors = [t for t in s.text if "ERROR" in t or "FAULT" in t]
        check("no script errors in this run", not errors, "; ".join(errors))
    finally:
        s.close()


def scenario_tilt(binary, repo):
    print("\nF. Tilt cutoff using AHRS_TRIM_X as a fake 10 degree roll, limit patched to 5")
    s = Sitl(binary, repo, main_lua({"local TILT_LIMIT_DEG   = 30": "local TILT_LIMIT_DEG   = 5"}))
    try:
        s.set_rc(thr=1800)
        s.arm()
        moved = s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 3)
        check("level: driving", s.armed and s.servo.get(1, 0) > 1600,
              f"moved={moved} armed={s.armed} FA={s.servo.get(1)} text={s.text[-3:]}")
        s.set_param("AHRS_TRIM_X", 0.17)
        tripped = s.wait(lambda: not s.armed and s.r(0) is False, 6)
        check("tilted: disarmed and R1 open", tripped is not None, f"{tripped}s")
        check("tilted: MAIN1/3 at 1500", (s.servo.get(1), s.servo.get(3)) == (1500, 1500))
        check("tilted: GCS message sent", any("TILT" in t for t in s.text))
        s.arm()
        s.pump(2.0)
        check("re-arm while still tilted is refused by the script", not s.armed and s.r(0) is False)
    finally:
        s.close()


def scenario_errors(binary, repo):
    print("\nG. Runtime errors inside the running script (patched copies)")
    s = Sitl(binary, repo, main_lua(inject='    if param:get("SCR_USER1") == 1 then error("injected") end'))
    try:
        s.set_rc(thr=1800)
        s.arm()
        s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 3)
        s.set_param("SCR_USER1", 1)
        safe = s.wait(lambda: not s.armed and s.r(0) is False and s.servo.get(1) == 1500, 3)
        check("Lua error: disarmed, R1 open, neutral", safe is not None, f"{safe}s")
        check("Lua error: GCS message", any("ERROR" in t for t in s.text), str(s.text[-3:]))
        s.set_param("SCR_USER1", 0)
        s.pump(1.5)
        s.arm()
        s.pump(BOOT_S + 1)
        check("Lua error: latched, cannot re-arm into motion", not s.armed and s.r(0) is False)
    finally:
        s.close()

    s = Sitl(binary, repo, main_lua(inject='    while param:get("SCR_USER1") == 2 do end'))
    try:
        s.set_rc(thr=1800)
        s.arm()
        s.wait(lambda: s.servo.get(1, 1500) > 1600, BOOT_S + 3)
        s.set_param("SCR_USER1", 2)
        fell = s.wait(lambda: s.servo.get(1) == 1500, 5)
        s.pump(2.0)
        check("runaway script: MAIN1 goes to 1500", fell is not None and s.servo.get(1) == 1500,
              f"{fell}s; armed={s.armed} R1={s.r(0)}; text={s.text[-3:]}")
    finally:
        s.close()


def scenario_failsafe(binary, repo):
    print("\nH. FS_ACTION 5 with radio loss (SIM_RC_FAIL), including in Hold via FS_OPTIONS")
    # FS_OPTIONS comes from mower.param.
    overrides = SIM_OVERRIDES.replace("FS_THR_ENABLE 0", "FS_THR_ENABLE 1")
    for label, mode in (("Manual", 0), ("Hold", 4)):
        s = Sitl(binary, repo, main_lua(), sim_overrides=overrides)
        try:
            s.arm()
            s.wait(lambda: s.armed and s.r(0) is True, 3)
            s.set_mode(mode)
            s.wait(lambda: s.mode == mode, 2)
            s.pump(BOOT_S)
            s.set_param("SIM_RC_FAIL", 1)
            done = s.wait(lambda: not s.armed, 6)
            off = s.wait(lambda: s.r(0) is False, 2)
            check(f"radio loss in {label}: disarms, then R1 opens", done is not None and off is not None,
                  f"disarm {done}s after loss, R1 {off}s later")
        finally:
            s.close()


def scenario_engine_relay(binary, repo):
    print("\nI. R2 engine relay: silent by default, doc's policy once confirmed")
    s = Sitl(binary, repo, main_lua())
    try:
        check("unconfirmed, boot: R2 off", s.r(1) is False, f"relay byte {s.relay_on}")
        s.arm()
        s.pump(BOOT_S + 1)
        check("unconfirmed, armed: R2 off", s.r(0) is True and s.r(1) is False, f"relay byte {s.relay_on}")
        s.disarm()
        s.pump(1)
        check("unconfirmed, disarmed: R2 off", s.r(1) is False, f"relay byte {s.relay_on}")
    finally:
        s.close()

    confirmed = {"local R2_WIRING_CONFIRMED = false": "local R2_WIRING_CONFIRMED = true"}
    s = Sitl(binary, repo, main_lua(confirmed))
    try:
        check("confirmed, boot: R2 energized before arming", s.r(1) is True and s.r(0) is False,
              f"relay byte {s.relay_on}")
        s.arm()
        s.wait(lambda: s.armed, 3)
        s.pump(BOOT_S + 1)
        check("confirmed, armed: R1 and R2 on", s.r(0) is True and s.r(1) is True, f"relay byte {s.relay_on}")
        s.disarm()
        off = s.wait(lambda: s.r(1) is False, 1)
        check("confirmed, disarm: R2 drops", off is not None, f"{off}s")
        s.pump(1)
        s.arm()
        s.pump(BOOT_S + 1)
        check("confirmed, re-arm: R1 closes, R2 stays latched off", s.r(0) is True and s.r(1) is False,
              f"relay byte {s.relay_on}")
    finally:
        s.close()

    extra = "RNGFND1_TYPE 10\nRNGFND1_ORIENT 0\nRNGFND1_MIN_CM 10\nRNGFND1_MAX_CM 600\nRNGFND1_MIN 0.1\nRNGFND1_MAX 6\n"
    s = Sitl(binary, repo, main_lua(confirmed), extra)
    try:
        s.rf_m = 5.0
        check("mission uploaded", s.upload_rover_mission())
        s.arm()
        s.wait(lambda: s.armed, 3)
        s.pump(BOOT_S + 1)
        # Auto needs a good EKF position, which takes a while after boot.
        auto = None
        for _ in range(30):
            s.set_mode(10)
            auto = s.wait(lambda: s.mode == 10, 2)
            if auto is not None:
                break
        check("Auto engaged", auto is not None, f"mode={s.mode}; {s.text[-2:]}")
        s.pump(1.0)
        check("Auto, 5 m: stays in Auto, R2 on", s.mode == 10 and s.r(1) is True, f"relay byte {s.relay_on}")
        s.rf_m = 1.0
        held = s.wait(lambda: s.mode == 4, 3)
        s.pump(0.1)
        check("Auto, 1 m: Hold, R2 dropped, neutral, still armed",
              held is not None and s.r(1) is False and s.servo.get(1) == 1500 and s.armed,
              f"{held}s, relay byte {s.relay_on}, FA={s.servo.get(1)}")
        s.rf_m = 5.0
        s.pump(1.0)
        check("obstacle gone: R2 stays latched off", s.r(1) is False)
    finally:
        s.close()


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    binary = os.path.abspath(sys.argv[1])
    # .../build/sitl/bin/ardurover, so the repo root is three levels above bin.
    repo = os.path.abspath(os.path.join(os.path.dirname(binary), "..", "..", ".."))
    for scenario in (scenario_fallback, scenario_params, scenario_sequence, scenario_rate,
                     scenario_rangefinder, scenario_tilt, scenario_errors, scenario_failsafe,
                     scenario_engine_relay):
        try:
            scenario(binary, repo)
        except Exception as exc:  # A crashed scenario must show as a failure.
            check(f"{scenario.__name__} crashed", False, repr(exc))
    failed = [r for r in results if not r[1]]
    print(f"\n{len(results)} checks, {len(failed)} failed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
