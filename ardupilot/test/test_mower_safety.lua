-- Logic tests for scripts/mower_safety.lua, using a small mock of the ArduPilot
-- bindings. Run from anywhere:  lua ardupilot/test/test_mower_safety.lua
--
-- This only tests the script's own logic against the mock. It says nothing about
-- real outputs, timing, or hardware. See ../README.md and sitl_check.py.

local here = arg[0]:match("^(.*)/[^/]*$") or "."
local file = assert(io.open(here .. "/../scripts/mower_safety.lua", "r"))
local SOURCE = file:read("a")
file:close()

local LOOP_MS, BOOT_MS, OPEN_MS = 10, 3000, 300
local R1, R2, R3 = 0, 1, 2
local MANUAL, HOLD, AUTO = 0, 4, 10

-- Builds the mock world, loads the script, and returns the world.
local function new_world(opts)
    opts = opts or {}
    local w = {
        clock = 1000, armed = false, mode = MANUAL, left = 1500, right = 1500,
        roll = 0, pitch = 0, relays = {}, relay_log = {}, out = {}, out_log = {}, msgs = {},
        scaled = {}, disarm_calls = 0, set_mode_ok = true,
        channels = {[73] = 4, [74] = 5, [94] = 0, [95] = 2}, relay_enabled = true,
        params = {SERVO1_TRIM = 1500, SERVO3_TRIM = 1500, SERVO5_TRIM = 1500, SERVO6_TRIM = 1500},
        estop = false, rf = opts.rf,
    }

    _G.millis = function() return {toint = function() return w.clock end} end
    _G.gcs = {send_text = function(_, sev, text) w.msgs[#w.msgs + 1] = {sev = sev, text = text} end}
    _G.arming = {
        is_armed = function() return w.armed end,
        disarm = function() w.disarm_calls = w.disarm_calls + 1; w.armed = false; return true end,
    }
    _G.vehicle = {
        get_mode = function() return w.mode end,
        set_mode = function(_, m)
            if not w.set_mode_ok then return false end
            w.mode = m
            return true
        end,
    }
    _G.relay = {
        on = function(_, i) w.relays[i] = true; w.relay_log[#w.relay_log + 1] = {i, true} end,
        off = function(_, i) w.relays[i] = false; w.relay_log[#w.relay_log + 1] = {i, false} end,
        enabled = function() return w.relay_enabled end,
    }
    _G.SRV_Channels = {
        get_output_pwm = function(_, f)
            if w.input_error then error("mock binding failure") end
            if f == 73 then return w.left end
            return w.right
        end,
        set_output_scaled = function(_, f, v) w.scaled[f] = v end,
        set_angle = function() end,
        set_output_pwm_chan_timeout = function(_, c, p, t)
            if w.output_error then error("mock output failure") end
            assert(math.type(p) == "integer", "PWM must be an integer")
            w.out[c] = p
            w.timeout = t
            w.out_log[#w.out_log + 1] = {c, p}
        end,
        find_channel = function(_, f) return w.channels[f] end,
        get_emergency_stop = function() return w.estop end,
    }
    _G.param = {get = function(_, n) return w.params[n] end}
    w.rc_pwm, w.rc_valid, w.rc_link = 1500, true, 1492
    _G.rc = {
        has_valid_input = function() return w.rc_valid end,
        get_pwm = function(_, ch) if ch == 5 then return w.rc_pwm end if ch == 2 then return w.rc_link end return 1500 end,
    }
    local attitude = {
        get_roll = function() return math.rad(w.roll) end,
        get_pitch = function() return math.rad(w.pitch) end,
    }
    if opts.new_ahrs then
        -- Rover 4.7 names. Deliberately error on the old ones to prove selection.
        attitude = {
            get_roll_rad = function() return math.rad(w.roll) end,
            get_pitch_rad = function() return math.rad(w.pitch) end,
            get_roll = function() error("deprecated name used") end,
            get_pitch = function() error("deprecated name used") end,
        }
    end
    _G.ahrs = attitude

    if opts.rf then
        local rf = {
            has_orientation = function() return w.rf.present end,
            status_orient = function() return w.rf.status end,
        }
        if opts.rf.metres then
            rf.distance_orient = function() return w.rf.dist_m end
        else
            rf.distance_cm_orient = function() return math.floor(w.rf.dist_m * 100) end
        end
        _G.rangefinder = rf
    else
        _G.rangefinder = opts.rf_binding_absent and nil or {has_orientation = function() return false end}
    end

    local src = SOURCE
    -- Logic tests use +1 signs, so expected outputs read as the doc formula. Pass
    -- field_signs = true to keep the signs measured on the mower.
    local patches = {}
    if not opts.field_signs then
        patches["local FORE_AFT_SIGN   = %-1"] = "local FORE_AFT_SIGN   = 1"
        patches["local DIFF_SIGN       = %-1"] = "local DIFF_SIGN       = 1"
    end
    for k, v in pairs(opts.patch or {}) do patches[k] = v end
    for pattern, replacement in pairs(patches) do
        local count
        src, count = src:gsub(pattern, replacement)
        assert(count == 1, "patch did not match: " .. pattern)
    end
    w.fn = assert(load(src, "mower_safety.lua"))()
    return w
end

-- Advances the clock, running the script's loop on its requested schedule.
local function run(w, ms)
    local stop = w.clock + ms
    while w.clock < stop do
        w.clock = w.clock + LOOP_MS
        local fn, delay = w.fn()
        assert(type(fn) == "function" and delay == LOOP_MS, "script must reschedule itself")
        w.fn = fn
    end
end

local function arm_and_boot(w)
    w.armed = true
    run(w, BOOT_MS + 2 * LOOP_MS)
end

local function neutral(w)
    return w.out[0] == 1500 and w.out[2] == 1500
end

local function has_msg(w, needle)
    for _, m in ipairs(w.msgs) do
        if m.text:find(needle, 1, true) then return true end
    end
    return false
end

local function commands_to(w, ...)
    local wanted = {}
    for _, i in ipairs({...}) do wanted[i] = true end
    local n = 0
    for _, e in ipairs(w.relay_log) do
        if wanted[e[1]] then n = n + 1 end
    end
    return n
end

-- Formula tests use the doc's +1 sign. The field value is checked separately.
local SIGN_PLUS = {["local DIFF_SIGN       = %-1"] = "local DIFF_SIGN       = 1",
                   ["local FORE_AFT_SIGN   = %-1"] = "local FORE_AFT_SIGN   = 1"}

local CONFIRMED = {["local R2_WIRING_CONFIRMED = false"] = "local R2_WIRING_CONFIRMED = true"}

local tests = {}
local function test(name, fn) tests[#tests + 1] = {name = name, fn = fn} end

-- Turns off both lifts (steering dead zone and per-track minimum), so outputs read as the
-- plain formula. Tests without it get the lifted values, computed by lifted() below.
local NO_LIFT = {["local STEER_DEADZONE_US  = 25"] = "local STEER_DEADZONE_US  = 0",
                 ["DRIVE.TRACK_MIN_US = 125"] = "DRIVE.TRACK_MIN_US = 0",
                 ["DRIVE.LIFT_FROM_US = 8 "] = "DRIVE.LIFT_FROM_US = 0 ",
                       ["DRIVE.PIVOT_MIN_US = 180"] = "DRIVE.PIVOT_MIN_US = 0"}
-- The script's per-track lift with its default constants, for expected values.
local TRACK_MIN, LIFT_FROM, PIVOT_MIN = 125, 8, 180
local function lifted(us, min)
    min = min or TRACK_MIN
    local a = math.abs(us)
    if a <= LIFT_FROM then return 0 end
    if a < min then a = min else return us end
    return us > 0 and a or -a
end
local function pwm(us) return math.floor(1500 + us + 0.5) end
local F1800 = pwm(lifted(300))  -- above the clamp, so still 1800
local NO_TRACK_LIFT = {["DRIVE.TRACK_MIN_US = 125"] = "DRIVE.TRACK_MIN_US = 0",
                       ["DRIVE.LIFT_FROM_US = 8 "] = "DRIVE.LIFT_FROM_US = 0 ",
                       ["DRIVE.PIVOT_MIN_US = 180"] = "DRIVE.PIVOT_MIN_US = 0"}
-- Formula tests jump straight between directions; the reverse hold has its own tests.
local NO_HOLD_PATTERN, NO_HOLD_VALUE = "DRIVE.REVERSE_HOLD_MS = 400", "DRIVE.REVERSE_HOLD_MS = 0"
local function no_hold(p)
    local t = {[NO_HOLD_PATTERN] = NO_HOLD_VALUE}
    for k, v in pairs(p or {}) do t[k] = v end
    return t
end

test("conversion matches the doc formula", function()
    local w = new_world({patch = no_hold(NO_LIFT)})
    arm_and_boot(w)
    for _, c in ipairs({
        {1500, 1500, 1500, 1500}, {1700, 1700, 1700, 1500}, {1300, 1300, 1300, 1500},
        {1700, 1300, 1500, 1700}, {1300, 1700, 1500, 1300}, {1900, 1500, 1700, 1700},
        {1100, 1900, 1500, 1100},
    }) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        assert(w.out[0] == c[3] and w.out[2] == c[4],
            ("L%d R%d gave FA%d D%d"):format(c[1], c[2], w.out[0], w.out[2]))
    end
end)

test("outputs clamp to 1000-2000", function()
    local w = new_world({patch = no_hold({["local FORE_AFT_GAIN   = 1.0"] = "local FORE_AFT_GAIN   = 5.0",
                                  ["local DIFF_GAIN       = 1.0"] = "local DIFF_GAIN       = 5.0",
                                  ["local DIFF_SIGN       = %-1"] = "local DIFF_SIGN       = 1",
                                  ["local FORE_AFT_SIGN   = %-1"] = "local FORE_AFT_SIGN   = 1"})})
    arm_and_boot(w)
    w.left, w.right = 2000, 1000
    run(w, LOOP_MS)
    assert(w.out[2] == 2000)
    w.left, w.right = 2000, 2000
    run(w, LOOP_MS)
    assert(w.out[0] == 2000)
    w.left, w.right = 1000, 1000
    run(w, LOOP_MS)
    assert(w.out[0] == 1000)
end)

test("field DIFF_SIGN -1: a right turn command gives differential below neutral", function()
    local w = new_world({field_signs = true})
    arm_and_boot(w)
    w.left, w.right = 1700, 1300
    run(w, LOOP_MS)
    assert(w.out[0] == 1500 and w.out[2] == pwm(-lifted(200, PIVOT_MIN)), "D" .. tostring(w.out[2]))
end)

test("field FORE_AFT_SIGN -1: a forward command gives fore-aft below neutral", function()
    local w = new_world({field_signs = true})
    arm_and_boot(w)
    w.left, w.right = 1700, 1700
    run(w, LOOP_MS)
    assert(w.out[0] == pwm(-lifted(200)) and w.out[2] == 1500)
end)

test("0, nil, and out-of-window inputs give neutral on both outputs", function()
    for _, bad in ipairs({0, 100, 3000}) do
        local w = new_world()
        arm_and_boot(w)
        w.left, w.right = 1800, bad
        run(w, LOOP_MS)
        assert(neutral(w), "bad value " .. bad)
    end
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = nil, 1800
    run(w, LOOP_MS)
    assert(neutral(w))
    assert(has_msg(w, "invalid L/R"))
end)

test("fallback setup: scaled 0 and a short timeout on every send", function()
    local w = new_world()
    run(w, LOOP_MS)
    assert(w.scaled[94] == 0 and w.scaled[95] == 0)
    assert(w.timeout > LOOP_MS and w.timeout <= 500)
    local before = #w.out_log
    run(w, 10 * LOOP_MS)
    assert(#w.out_log - before == 20, "must send both channels every loop while disarmed")
end)

test("no motion while disarmed, relay open", function()
    local w = new_world()
    w.left, w.right = 1900, 1900
    run(w, 500)
    assert(neutral(w) and w.relays[R1] == false)
end)

test("arm: R1 closes at once, neutral through boot delay, then motion", function()
    local w = new_world()
    w.left, w.right = 1800, 1800
    w.armed = true
    run(w, LOOP_MS)
    assert(w.relays[R1] == true)
    run(w, BOOT_MS - 3 * LOOP_MS)
    assert(neutral(w), "must hold neutral during boot delay")
    run(w, 4 * LOOP_MS)
    assert(w.out[0] == F1800, "motion allowed after boot delay")
end)

test("normal disarm: neutral first, R1 opens after the delay", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, LOOP_MS)
    w.armed = false
    run(w, LOOP_MS)
    assert(neutral(w), "neutral on the first disarmed cycle")
    assert(w.relays[R1] == true, "R1 still closed during the delay")
    run(w, OPEN_MS + LOOP_MS)
    assert(w.relays[R1] == false, "R1 open after the delay")
end)

test("re-arm during the disarm delay restarts the boot delay", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    w.armed = false
    run(w, 2 * LOOP_MS)
    w.armed = true
    run(w, 5 * LOOP_MS)
    assert(neutral(w) and w.relays[R1] == true)
    run(w, BOOT_MS)
    assert(w.out[0] == F1800)
end)

test("script start while already armed still waits and goes neutral", function()
    local w = new_world()
    w.armed = true
    w.left, w.right = 1800, 1800
    run(w, 100)
    assert(neutral(w) and w.relays[R1] == true)
end)

test("emergency stop flag forces neutral", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    w.estop = true
    run(w, LOOP_MS)
    assert(neutral(w))
end)

test("Hold forces neutral at once, even while L/R are still ramping down", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, LOOP_MS)
    assert(w.out[0] == F1800)
    w.mode = HOLD
    run(w, LOOP_MS)
    assert(neutral(w) and w.armed and w.relays[R1] == true)
end)

test("rangefinder stop is neutral on the same cycle as the trip", function()
    local w = new_world({rf = {present = true, metres = true, status = 4, dist_m = 1}})
    w.mode = AUTO
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    while w.mode == AUTO do
        run(w, LOOP_MS)
        assert(w.mode == AUTO or neutral(w), "outputs must be neutral as soon as the mode is Hold")
    end
end)

test("tilt: a short spike does not trip", function()
    local w = new_world()
    arm_and_boot(w)
    w.roll = 40
    run(w, 200)
    w.roll = 0
    run(w, 1000)
    assert(w.armed and w.disarm_calls == 0)
end)

test("tilt: sustained trip disarms and opens R1 with no disarm delay", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    w.pitch = -40
    run(w, 600)
    assert(not w.armed and w.disarm_calls >= 1)
    assert(w.relays[R1] == false and neutral(w))
    assert(has_msg(w, "TILT"))
end)

test("tilt: NaN attitude counts as over the limit", function()
    local w = new_world()
    arm_and_boot(w)
    w.roll = 0/0
    run(w, 600)
    assert(not w.armed and w.relays[R1] == false)
end)

test("tilt: stays latched while tilted, clears only when level and disarmed", function()
    local w = new_world()
    arm_and_boot(w)
    w.roll = 40
    run(w, 600)
    w.armed = true
    run(w, 100)
    assert(not w.armed, "re-arming while latched is disarmed again")
    w.roll = 0
    run(w, 2100)
    w.armed = true
    run(w, BOOT_MS + 100)
    assert(w.armed and w.relays[R1] == true, "normal again once cleared")
end)

test("rangefinder: absent in every form does nothing", function()
    for _, o in ipairs({{}, {rf_binding_absent = true}, {rf = {present = false, metres = true, status = 0, dist_m = 0}}}) do
        local w = new_world(o)
        w.mode = AUTO
        arm_and_boot(w)
        w.left, w.right = 1800, 1800
        run(w, 1000)
        assert(w.mode == AUTO and w.armed and w.out[0] == F1800)
    end
end)

for _, version in ipairs({{metres = true, name = "4.7 distance_orient"}, {metres = false, name = "4.6 distance_cm_orient"}}) do
    test("rangefinder: debounced trip to Hold, " .. version.name, function()
        local rf = {present = true, metres = version.metres, status = 4, dist_m = 5}
        local w = new_world({rf = rf})
        w.mode = AUTO
        arm_and_boot(w)
        run(w, 500)
        assert(w.mode == AUTO, "5 m must not trip")
        w.rf.dist_m = 1.5
        run(w, 150)
        assert(w.mode == AUTO, "must debounce")
        w.rf.dist_m = 5
        run(w, 150)
        w.rf.dist_m = 1.5
        run(w, 350)
        assert(w.mode == HOLD and w.armed, "sustained reading trips to Hold, stays armed")
        assert(has_msg(w, "obstacle"))
    end)
end

test("rangefinder: unhealthy or out of range high does not trip", function()
    for _, status in ipairs({0, 1, 3}) do
        local w = new_world({rf = {present = true, metres = true, status = status, dist_m = 0.5}})
        w.mode = AUTO
        arm_and_boot(w)
        run(w, 500)
        assert(w.mode == AUTO, "status " .. status)
    end
end)

test("rangefinder: below sensor minimum trips", function()
    local w = new_world({rf = {present = true, metres = true, status = 2, dist_m = 0}})
    w.mode = AUTO
    arm_and_boot(w)
    run(w, 400)
    assert(w.mode == HOLD)
end)

test("rangefinder: Hold refused means disarm", function()
    local w = new_world({rf = {present = true, metres = true, status = 4, dist_m = 1}})
    w.mode = AUTO
    w.set_mode_ok = false
    arm_and_boot(w)
    run(w, 400)
    assert(not w.armed and w.disarm_calls >= 1)
end)

test("rangefinder: Manual trips on forward only, so the operator can reverse away", function()
    local w = new_world({rf = {present = true, metres = true, status = 4, dist_m = 1}})
    arm_and_boot(w)
    w.left, w.right = 1300, 1300
    run(w, 500)
    assert(w.mode == MANUAL and w.out[0] == pwm(-lifted(200)), "reverse allowed")
    w.left, w.right = 1800, 1800
    run(w, 400)
    assert(w.mode == HOLD, "forward trips")
end)

test("R2 unconfirmed: R2 and the spare relays get no command in any path", function()
    local w = new_world({rf = {present = true, metres = true, status = 4, dist_m = 5}})
    w.mode = AUTO
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 200)
    w.rf.dist_m = 1
    run(w, 400)
    w.rf.dist_m = 5
    w.armed = false
    run(w, 500)
    w.armed = true
    run(w, BOOT_MS)
    w.roll = 40
    run(w, 600)
    w.roll = 0
    w.input_error = true
    run(w, 100)
    for _, e in ipairs(w.relay_log) do
        assert(e[1] == R1, "only R1 may be commanded, saw relay index " .. e[1])
    end
    assert(commands_to(w, R1) > 0)
end)

test("R2 confirmed: energized after boot, before arming, and stays on if never armed", function()
    local w = new_world({patch = CONFIRMED})
    run(w, LOOP_MS)
    assert(w.relays[R2] == true and w.relays[R1] == false)
    run(w, 5000)
    assert(w.relays[R2] == true)
end)

test("R2 confirmed: stays on while armed and driving", function()
    local w = new_world({patch = CONFIRMED})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 500)
    assert(w.relays[R2] == true)
end)

test("R2 confirmed: normal disarm drops R2 at once, R1 still waits, latched after", function()
    local w = new_world({patch = CONFIRMED})
    arm_and_boot(w)
    w.armed = false
    run(w, LOOP_MS)
    assert(w.relays[R2] == false, "R2 drops on the first disarmed cycle")
    assert(w.relays[R1] == true, "R1 still waits for its delay")
    run(w, OPEN_MS + LOOP_MS)
    assert(w.relays[R1] == false)
    assert(has_msg(w, "engine killed"))
    w.armed = true
    run(w, BOOT_MS + 100)
    assert(w.relays[R2] == false, "latched through a re-arm")
    assert(w.relays[R1] == true, "tracks still work when re-armed")
end)

test("R2 confirmed: tilt kills the engine, and it stays killed after tilt clears", function()
    local w = new_world({patch = CONFIRMED})
    arm_and_boot(w)
    w.roll = 40
    run(w, 600)
    assert(w.relays[R2] == false)
    w.roll = 0
    run(w, 3000)
    assert(w.relays[R2] == false)
end)

test("R2 confirmed: rangefinder stop gives Hold and engine kill, stays armed", function()
    local w = new_world({patch = CONFIRMED, rf = {present = true, metres = true, status = 4, dist_m = 5}})
    w.mode = AUTO
    arm_and_boot(w)
    assert(w.relays[R2] == true)
    w.rf.dist_m = 1
    run(w, 400)
    assert(w.mode == HOLD and w.relays[R2] == false and w.armed and w.relays[R1] == true)
    w.rf.dist_m = 5
    run(w, 500)
    assert(w.relays[R2] == false, "latched after the obstacle clears")
end)

test("R2 confirmed: Manual reverse near an obstacle does not kill the engine", function()
    local w = new_world({patch = CONFIRMED, rf = {present = true, metres = true, status = 4, dist_m = 1}})
    arm_and_boot(w)
    w.left, w.right = 1300, 1300
    run(w, 500)
    assert(w.relays[R2] == true)
end)

test("R2 confirmed: runtime error kills the engine", function()
    local w = new_world({patch = CONFIRMED})
    arm_and_boot(w)
    w.input_error = true
    run(w, 3 * LOOP_MS)
    assert(w.relays[R2] == false and w.relays[R1] == false)
    w.input_error = false
    run(w, 500)
    assert(w.relays[R2] == false)
end)

test("R2 confirmed: bad config at boot never energizes R2", function()
    local w = new_world({patch = CONFIRMED})
    w.params.SERVO1_TRIM = 1100
    run(w, 500)
    for _, e in ipairs(w.relay_log) do
        assert(not (e[1] == R2 and e[2]), "R2 was energized")
    end
end)

test("runtime error: safe state, GCS message, loop keeps running, fault latched", function()
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, LOOP_MS)
    assert(w.out[0] == F1800)
    w.input_error = true
    run(w, 3 * LOOP_MS)
    assert(neutral(w) and w.relays[R1] == false and not w.armed)
    assert(has_msg(w, "ERROR"))
    w.input_error = false
    w.armed = true
    run(w, BOOT_MS + 100)
    assert(not w.armed and w.relays[R1] == false, "latched until reboot")
end)

test("error while writing outputs: relays and disarm still run", function()
    local w = new_world()
    arm_and_boot(w)
    w.output_error = true
    run(w, 3 * LOOP_MS)
    assert(w.relays[R1] == false and not w.armed)
end)

test("GCS messages are rate limited", function()
    local w = new_world()
    arm_and_boot(w)
    w.input_error = true
    local before = #w.msgs
    run(w, 1000)
    local sent = #w.msgs - before
    assert(sent >= 1 and sent <= 2, "sent " .. sent .. " in 1 s")
end)

test("bad config faults and refuses motion", function()
    local cases = {
        function(w) w.relay_enabled = false end,
        function(w) w.channels[94] = 1 end,
        function(w) w.channels[95] = nil end,
        function(w) w.channels[73] = 0 end,
        function(w) w.params.SERVO1_TRIM = 1100 end,
        function(w) w.params.SERVO6_TRIM = nil end,
    }
    for i, breakit in ipairs(cases) do
        local w = new_world()
        breakit(w)
        w.armed = true
        w.left, w.right = 1800, 1800
        run(w, BOOT_MS + 100)
        assert(not w.armed and neutral(w) and w.relays[R1] == false, "case " .. i)
        assert(has_msg(w, "FAULT"), "case " .. i)
    end
end)

test("Rover 4.7 attitude names are preferred when present", function()
    local w = new_world({new_ahrs = true})
    arm_and_boot(w)
    run(w, 100)
    assert(w.armed, "deprecated binding would have faulted")
    w.roll = 40
    run(w, 600)
    assert(not w.armed)
end)

local STARTER = {
    ["local R2_WIRING_CONFIRMED = false"] = "local R2_WIRING_CONFIRMED = true",
    ["local STARTER_WIRING_CONFIRMED = false"] = "local STARTER_WIRING_CONFIRMED = true",
}

test("starter: no R3 command while unconfirmed", function()
    local w = new_world({patch = CONFIRMED})
    w.rc_pwm = 1000
    run(w, 1000)
    assert(commands_to(w, R3) == 0)
end)

test("starter: needs R2 confirmed, else fault", function()
    local w = new_world({patch = {["local STARTER_WIRING_CONFIRMED = false"] = "local STARTER_WIRING_CONFIRMED = true"}})
    run(w, 100)
    assert(has_msg(w, "starter needs R2"))
    assert(w.relays[R3] ~= true)
end)

test("starter: cranks only while held, off on release", function()
    local w = new_world({patch = STARTER})
    run(w, 100)
    assert(w.relays[R3] == false)
    w.rc_pwm = 1000
    run(w, 500)
    assert(w.relays[R3] == true, "cranking while held")
    w.rc_pwm = 1500
    run(w, 2 * LOOP_MS)
    assert(w.relays[R3] == false, "off on release")
end)

test("starter: held at boot does not crank until released", function()
    local w = new_world({patch = STARTER})
    w.rc_pwm = 1000
    run(w, 1000)
    assert(w.relays[R3] ~= true)
    w.rc_pwm = 1500
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 100)
    assert(w.relays[R3] == true)
end)

test("starter: capped at 2.5 s, needs release and rest before next crank", function()
    local w = new_world({patch = STARTER})
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 2400)
    assert(w.relays[R3] == true)
    run(w, 200)
    assert(w.relays[R3] == false, "cap")
    run(w, 6000)
    assert(w.relays[R3] == false, "still held: no re-crank")
    w.rc_pwm = 1500
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 100)
    assert(w.relays[R3] == true, "rest already passed")
    w.rc_pwm = 1500
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 1000)
    assert(w.relays[R3] == false, "inside rest window")
end)

test("starter: never while armed, stops on arm", function()
    local w = new_world({patch = STARTER})
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 200)
    assert(w.relays[R3] == true)
    w.armed = true
    run(w, 2 * LOOP_MS)
    assert(w.relays[R3] == false)
    w.rc_pwm = 1500
    run(w, 6000)
    w.rc_pwm = 1000
    run(w, 500)
    assert(w.relays[R3] == false, "armed")
end)

test("starter: never after engine kill latch, or on radio loss", function()
    local w = new_world({patch = STARTER})
    arm_and_boot(w)
    w.armed = false
    run(w, 1000)
    w.rc_pwm = 1500
    run(w, 6000)
    w.rc_pwm = 1000
    run(w, 500)
    assert(w.relays[R3] == false, "killed latch")
    local v = new_world({patch = STARTER})
    run(v, 100)
    v.rc_pwm = 1000
    run(v, 200)
    assert(v.relays[R3] == true)
    v.rc_valid = false
    run(v, 2 * LOOP_MS)
    assert(v.relays[R3] == false, "radio loss")
end)

test("starter: fault and tilt open R3", function()
    local w = new_world({patch = STARTER})
    run(w, 100)
    w.rc_pwm = 1000
    run(w, 200)
    w.roll = 45
    run(w, 2 * LOOP_MS)
    assert(w.relays[R3] == false, "tilt")
    local v = new_world({patch = STARTER})
    run(v, 100)
    v.rc_pwm = 1000
    run(v, 200)
    v.input_error = true
    v.channels[94] = 1
    run(v, 1100)
    assert(v.relays[R3] == false, "fault")
end)

test("starter: radio values of 0 do not count as held", function()
    local w = new_world({patch = STARTER})
    w.rc_pwm = 0
    run(w, 500)
    assert(w.relays[R3] ~= true)
end)

test("radio: CH2 at 995 while armed disarms after 0.2 s", function()
    local w = new_world()
    arm_and_boot(w)
    w.rc_link = 995
    run(w, 100)
    assert(w.armed, "too fast")
    run(w, 150)
    assert(not w.armed and has_msg(w, "radio lost"))
end)

test("radio: no valid RC input disarms", function()
    local w = new_world()
    arm_and_boot(w)
    w.rc_valid = false
    run(w, 300)
    assert(not w.armed)
end)

test("radio: a short CH2 glitch does not disarm", function()
    local w = new_world()
    arm_and_boot(w)
    w.rc_link = 995
    run(w, 100)
    w.rc_link = 1492
    run(w, 500)
    assert(w.armed)
end)

test("radio: arming while lost is undone", function()
    local w = new_world()
    w.rc_link = 995
    run(w, 500)
    w.armed = true
    run(w, 2 * LOOP_MS)
    assert(not w.armed)
end)

test("radio: lost blocks the starter", function()
    local w = new_world({patch = STARTER})
    run(w, 100)
    w.rc_link = 995
    run(w, 300)
    w.rc_pwm = 1000
    run(w, 300)
    assert(w.relays[R3] ~= true)
end)

test("radio: the full live CH2 range counts as linked", function()
    for _, v in ipairs({1070, 1492, 1920}) do
        local w = new_world()
        w.rc_link = v
        arm_and_boot(w)
        run(w, 500)
        assert(w.armed, "CH2 " .. v)
    end
end)

test("convert: counter-rotating request becomes a pure pivot", function()
    local w = new_world({patch = no_hold(NO_LIFT)})
    arm_and_boot(w)
    w.left, w.right = 1395, 1999
    run(w, LOOP_MS)
    assert(w.out[0] == 1500, "fore " .. w.out[0])
    assert(w.out[2] == 1198 or w.out[2] == 1199, "diff " .. w.out[2])
    w.left, w.right = 1700, 1600
    run(w, LOOP_MS)
    assert(w.out[0] == 1650 and w.out[2] == 1550, "same-direction passes through")
end)

test("radio: lost in Auto keeps going, other modes disarm", function()
    local w = new_world()
    w.mode = AUTO
    arm_and_boot(w)
    w.rc_link = 995
    run(w, 500)
    assert(w.armed, "Auto should continue")
    w.mode = MANUAL
    run(w, 3 * LOOP_MS)
    assert(not w.armed, "Manual should disarm")
end)

-- Changed with the per-track clamp: the small pivot (1520/1480) is now clamped to 125 us.
test("steering dead-zone lift applies only while driving", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    w.left, w.right = 1930, 1870   -- driving forward, df = +30
    run(w, LOOP_MS)
    assert(w.out[2] == pwm(25 + 30 * 475 / 500), "lifted diff " .. w.out[2])
    w.left, w.right = 1908, 1892   -- df = +8, below the steering lift threshold
    run(w, LOOP_MS)
    assert(w.out[2] == 1508, "tiny diff passes " .. w.out[2])
    w.left, w.right = 1520, 1480   -- a small pivot: no steering lift, but the track lift applies
    run(w, LOOP_MS)
    assert(w.out[0] == 1500 and w.out[2] == pwm(lifted(20, PIVOT_MIN)), "small pivot " .. w.out[2])
end)

test("steering lift never reverses the inner track", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    -- Old order (steering lift before any track lift) sent the inner track to -17 here.
    for _, c in ipairs({{1615, 1505}, {1560, 1500}, {1700, 1500}, {1385, 1495}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        local fa, df = w.out[0] - 1500, w.out[2] - 1500
        assert(math.abs(df) <= math.abs(fa) + 1, ("L%d R%d gave FA%d D%d"):format(c[1], c[2], w.out[0], w.out[2]))
    end
end)

-- Runs ms of loops and returns how long the outputs were neutral, counted from the start.
local function neutral_ms(w, ms)
    local n, t = 0, 0
    while t < ms do
        run(w, LOOP_MS)
        t = t + LOOP_MS
        if neutral(w) then n = n + LOOP_MS else break end
    end
    return n
end

local HOLD_MS = 400

test("reverse hold: forward into a right pivot (right track flips) holds neutral, then pivots", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 500)
    assert(w.out[0] == 1800)
    w.left, w.right = 1999, 1001
    local n = neutral_ms(w, 1000)
    assert(n >= HOLD_MS and n <= HOLD_MS + 2 * LOOP_MS, "held " .. n)
    assert(w.out[0] == 1500 and w.out[2] >= 1990, "pivot after hold, D" .. w.out[2])
    run(w, 1000)
    assert(w.out[2] >= 1990, "no further hold while pivoting")
end)

test("reverse hold: pivot left into pivot right (both flip) holds", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1200, 1800
    run(w, 300)
    assert(w.out[2] == 1200, "pivot left from standstill, no hold")
    w.left, w.right = 1800, 1200
    local n = neutral_ms(w, 1000)
    assert(n >= HOLD_MS, "held " .. n)
    assert(w.out[2] == 1800)
end)

test("reverse hold: forward into reverse holds", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1700, 1700
    run(w, 300)
    w.left, w.right = 1300, 1300
    assert(neutral_ms(w, 1000) >= HOLD_MS)
    assert(w.out[0] == 1300)
end)

test("reverse hold: none from standstill or for slowing in the same direction", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1999, 1001
    run(w, LOOP_MS)
    assert(not neutral(w), "pivot from standstill must start at once")
    w.left, w.right = 1500, 1500
    run(w, HOLD_MS + LOOP_MS)
    w.left, w.right = 1001, 1999
    run(w, LOOP_MS)
    assert(w.out[2] == 1001, "neutral long enough: opposite pivot needs no hold")
    local v = new_world({patch = NO_LIFT})
    arm_and_boot(v)
    v.left, v.right = 1900, 1900
    run(v, 300)
    for _, c in ipairs({{1700, 1700}, {1550, 1550}, {1900, 1700}, {1700, 1530}}) do
        v.left, v.right = c[1], c[2]
        run(v, LOOP_MS)
        assert(not neutral(v), ("no hold for L%d R%d"):format(c[1], c[2]))
    end
end)

test("reverse hold: a short stop is not enough, the flip still holds", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 300)
    w.left, w.right = 1500, 1500
    run(w, 100)
    w.left, w.right = 1999, 1001
    assert(neutral_ms(w, 1000) >= HOLD_MS)
end)

test("reverse hold: flipping back during the hold finishes it once, no oscillation", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 300)
    w.left, w.right = 1999, 1001
    run(w, 200)
    assert(neutral(w))
    w.left, w.right = 1800, 1800
    local n = neutral_ms(w, 1000)
    assert(n >= HOLD_MS - 200 - LOOP_MS and n <= HOLD_MS - 200 + LOOP_MS, "rest of hold " .. n)
    assert(w.out[0] == 1800)
    -- Alternating every 100 ms: never longer than one hold in neutral, and motion between.
    local moving = 0
    for i = 1, 20 do
        if i % 2 == 0 then w.left, w.right = 1800, 1800 else w.left, w.right = 1999, 1001 end
        run(w, 100)
        if not neutral(w) then moving = moving + 1 end
    end
    assert(moving >= 3, "drives between holds, moving samples " .. moving)
end)

test("reverse hold: Hold mode and disarm during the hold are neutral at once", function()
    local w = new_world({patch = NO_LIFT})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 300)
    w.left, w.right = 1999, 1001
    run(w, 100)
    w.mode = HOLD
    run(w, LOOP_MS)
    assert(neutral(w) and w.armed)
    w.mode = MANUAL
    w.left, w.right = 1800, 1800
    run(w, 500)
    w.left, w.right = 1999, 1001
    run(w, 100)
    w.armed = false
    run(w, LOOP_MS)
    assert(neutral(w))
    run(w, OPEN_MS + LOOP_MS)
    assert(neutral(w) and w.relays[R1] == false)
end)

test("reverse hold: field signs, forward into a right pivot", function()
    local w = new_world({field_signs = true})
    arm_and_boot(w)
    w.left, w.right = 1800, 1800
    run(w, 300)
    assert(w.out[0] == pwm(-lifted(300)), "field forward is below neutral")
    w.left, w.right = 1999, 1001
    assert(neutral_ms(w, 1000) >= HOLD_MS)
    assert(w.out[0] == 1500 and w.out[2] <= 1010, "field right pivot D" .. w.out[2])
end)

-- Per-track minimum command (DRIVE.TRACK_MIN_US / DRIVE.LIFT_FROM_US).
-- Returns the left and right track commands the box acts on, from the +1-sign outputs.
local function tracks(w)
    local fa, df = w.out[0] - 1500, w.out[2] - 1500
    return fa + df, fa - df
end

test("track lift: tiny commands are neutral", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    for _, c in ipairs({{1508, 1508}, {1492, 1492}, {1505, 1495}, {1500, 1508}, {1493, 1507}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        assert(neutral(w), ("L%d R%d gave FA%d D%d"):format(c[1], c[2], w.out[0], w.out[2]))
    end
end)

test("track lift: just above the threshold gives 125 us, above 125 us unchanged", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    for _, c in ipairs({{1509, 1625, 1500}, {1491, 1375, 1500}, {1624, 1625, 1500}, {1626, 1626, 1500},
                        {1800, 1800, 1500}, {2000, 2000, 1500}, {1000, 1000, 1500}}) do
        w.left, w.right = c[1], c[1]
        run(w, LOOP_MS)
        assert(w.out[0] == c[2] and w.out[2] == c[3], ("L/R %d gave FA%d"):format(c[1], w.out[0]))
    end
    w.left, w.right = 2000, 1000
    run(w, LOOP_MS)
    assert(w.out[0] == 1500 and w.out[2] == 2000, "full pivot unchanged D" .. w.out[2])
    w.left, w.right = 1600, 1600   -- MOT_THR_MIN 20 (100 us) is a common input
    run(w, LOOP_MS)
    assert(w.out[0] == 1625, "100 us clamped up FA" .. w.out[0])
end)

test("track lift: monotonic and continuous above the threshold, both directions", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    for _, sign in ipairs({1, -1}) do
        local prev = 0
        for us = 0, 500 do
            w.left, w.right = 1500 + sign * us, 1500 + sign * us
            run(w, LOOP_MS)
            local out = sign * (w.out[0] - 1500)
            assert(out >= prev, ("not monotonic at %d: %d after %d"):format(sign * us, out, prev))
            if us > LIFT_FROM + 1 then
                assert(out - prev <= 2, ("jump at %d: %d to %d"):format(sign * us, prev, out))
            end
            assert(us <= LIFT_FROM and out == 0 or out >= TRACK_MIN, "inside the deadband at " .. us)
            prev = out
        end
        assert(prev == 500, "full scale " .. prev)
    end
end)

test("track lift: small pivots lifted to opposite >= 125 us on both tracks, fore-aft 0", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    for _, c in ipairs({{1490, 1510}, {1530, 1470}, {1450, 1550}, {1400, 1600}, {1395, 1610}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        local l, r = tracks(w)
        assert(w.out[0] == 1500, ("L%d R%d FA%d"):format(c[1], c[2], w.out[0]))
        assert(l == -r and math.abs(l) >= TRACK_MIN, ("L%d R%d tracks %d %d"):format(c[1], c[2], l, r))
        assert((l > 0) == (c[1] > c[2]), "pivot direction kept")
    end
end)

test("track lift: forward with slight steering stays forward on both tracks, lifted", function()
    local w = new_world({patch = no_hold()})
    arm_and_boot(w)
    -- Below the clamp both tracks can land on 125 (l == r): the clamp loses small differences.
    for _, c in ipairs({{1610, 1600}, {1520, 1512}, {1640, 1580}, {1612, 1588}, {1900, 1880}, {1700, 1630}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        local l, r = tracks(w)
        assert(l >= TRACK_MIN and r >= TRACK_MIN and l >= r, ("L%d R%d tracks %d %d"):format(c[1], c[2], l, r))
    end
    -- A track barely reversed (inside PIVOT_DEADBAND) counts as stopped, not lifted backward.
    for _, c in ipairs({{1600, 1490}, {1600, 1485}, {1515, 1400}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        local l, r = tracks(w)
        -- PWM rounding of fa and df can leave 1 us on the stopped track.
        assert((l * r >= 0 or math.min(math.abs(l), math.abs(r)) <= 1) and math.max(math.abs(l), math.abs(r)) >= TRACK_MIN,
            ("L%d R%d tracks %d %d"):format(c[1], c[2], l, r))
    end
end)

test("track lift: steering difference at cruise passes through 1:1", function()
    -- Track clamp alone (steering lift off): a cruise difference is untouched.
    local w = new_world({patch = no_hold({["local STEER_DEADZONE_US  = 25"] = "local STEER_DEADZONE_US  = 0"})})
    arm_and_boot(w)
    for _, c in ipairs({{1900, 1800}, {1800, 1900}, {1750, 1640}, {1200, 1300}, {1626, 1700}}) do
        w.left, w.right = c[1], c[2]
        run(w, LOOP_MS)
        local l, r = tracks(w)
        assert(l == c[1] - 1500 and r == c[2] - 1500, ("L%d R%d tracks %d %d"):format(c[1], c[2], l, r))
    end
    -- Defaults: whenever both tracks end up at or above 125 us, the output is exactly what
    -- it was before the track clamp existed (steering lift included).
    local cases = {{1900, 1800}, {1930, 1870}, {1908, 1892}, {1800, 1700}, {1300, 1150}, {1999, 1001}}
    local function outputs(patch)  -- worlds share the mock globals, so run one at a time
        local w2, got = new_world({patch = no_hold(patch)}), {}
        arm_and_boot(w2)
        for i, c in ipairs(cases) do
            w2.left, w2.right = c[1], c[2]
            run(w2, LOOP_MS)
            got[i] = w2.out[0] .. "/" .. w2.out[2]
        end
        return got
    end
    local with, without = outputs(nil), outputs(NO_TRACK_LIFT)
    for i, c in ipairs(cases) do
        assert(with[i] == without[i], ("L%d R%d: %s vs %s"):format(c[1], c[2], with[i], without[i]))
    end
end)

test("track lift: field signs", function()
    local w = new_world({field_signs = true, patch = no_hold()})
    arm_and_boot(w)
    w.left, w.right = 1600, 1600
    run(w, LOOP_MS)
    assert(w.out[0] == pwm(-lifted(100)) and w.out[0] <= 1500 - TRACK_MIN and w.out[2] == 1500, "forward FA" .. w.out[0])
    w.left, w.right = 1530, 1470   -- small right pivot: field differential below neutral
    run(w, LOOP_MS)
    assert(w.out[0] == 1500 and w.out[2] == pwm(-lifted(30, PIVOT_MIN)) and w.out[2] <= 1500 - PIVOT_MIN, "pivot D" .. w.out[2])
    w.left, w.right = 1505, 1505
    run(w, LOOP_MS)
    assert(neutral(w))
end)

test("track lift: reverse hold sees the lifted directions", function()
    -- A 10 us track was below DIR_DEADBAND before the lift; lifted it is driven, so a flip holds.
    local w = new_world()
    arm_and_boot(w)
    w.left, w.right = 1510, 1510
    run(w, 300)
    assert(w.out[0] == pwm(lifted(10)))
    w.left, w.right = 1530, 1470
    assert(neutral_ms(w, 1000) >= HOLD_MS, "small forward into small pivot holds")
    assert(w.out[0] == 1500 and w.out[2] == pwm(lifted(30, PIVOT_MIN)))
    -- Commands at or below LIFT_FROM_US never drive, so they never cause a hold.
    local v = new_world()
    arm_and_boot(v)
    v.left, v.right = 1506, 1506
    run(v, 300)
    v.left, v.right = 1300, 1300
    run(v, LOOP_MS)
    assert(v.out[0] == pwm(-lifted(200)), "no hold after a tiny command")
    -- A barely-reversed track is zeroed before the hold sees it: no flip, no hold.
    local u = new_world()
    arm_and_boot(u)
    u.left, u.right = 1700, 1700
    run(u, 300)
    u.left, u.right = 1700, 1490
    run(u, LOOP_MS)
    assert(not neutral(u), "no hold for a track inside PIVOT_DEADBAND")
end)

test("ArduPilot Lua limit: under 100 top-level locals", function()
    local n = 0
    for line in SOURCE:gmatch("[^\n]+") do
        if line:match("^local ") then n = n + 1 end
    end
    -- ArduPilot's embedded Lua stops at 100 in the main chunk; desktop Lua allows 200.
    assert(n <= 90, n .. " top-level locals, keep a margin under 100")
end)

local failed = 0
for _, t in ipairs(tests) do
    local ok, err = pcall(t.fn)
    print((ok and "ok    " or "FAIL  ") .. t.name .. (ok and "" or "\n      " .. tostring(err)))
    if not ok then failed = failed + 1 end
end
print(("%d tests, %d failed"):format(#tests, failed))
os.exit(failed == 0 and 0 or 1)
