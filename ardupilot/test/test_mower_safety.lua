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
local R1, R2 = 0, 1
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
    for pattern, replacement in pairs(opts.patch or {}) do
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

local CONFIRMED = {["local R2_WIRING_CONFIRMED = false"] = "local R2_WIRING_CONFIRMED = true"}

local tests = {}
local function test(name, fn) tests[#tests + 1] = {name = name, fn = fn} end

test("conversion matches the doc formula", function()
    local w = new_world()
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
    local w = new_world({patch = {["local FORE_AFT_GAIN   = 1.0"] = "local FORE_AFT_GAIN   = 5.0",
                                  ["local DIFF_GAIN       = 1.0"] = "local DIFF_GAIN       = 5.0"}})
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

test("sign constants flip the output", function()
    local w = new_world({patch = {["local FORE_AFT_SIGN   = 1"] = "local FORE_AFT_SIGN   = -1"}})
    arm_and_boot(w)
    w.left, w.right = 1700, 1700
    run(w, LOOP_MS)
    assert(w.out[0] == 1300)
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
    assert(w.out[0] == 1800, "motion allowed after boot delay")
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
    assert(w.out[0] == 1800)
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
    assert(w.out[0] == 1800)
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
        assert(w.mode == AUTO and w.armed and w.out[0] == 1800)
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
    assert(w.mode == MANUAL and w.out[0] == 1300, "reverse allowed")
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
    assert(w.out[0] == 1800)
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

local failed = 0
for _, t in ipairs(tests) do
    local ok, err = pcall(t.fn)
    print((ok and "ok    " or "FAIL  ") .. t.name .. (ok and "" or "\n      " .. tostring(err)))
    if not ok then failed = failed + 1 end
end
print(("%d tests, %d failed"):format(#tests, failed))
os.exit(failed == 0 and 0 or 1)
