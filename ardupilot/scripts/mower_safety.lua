-- mower_safety.lua: drive conversion and safety interlocks for the tracked mower.
--
-- STATUS: DRAFT. NEVER RUN ON REAL HARDWARE. See ../README.md for what has been
-- tested (simulator and mock only) and the bench-test order. This machine has a
-- blade. This script is not a safety system on its own: keep the key within reach.
--
-- Targets ArduPilot Rover 4.6 and later. Bindings whose names changed between
-- versions (ahrs roll/pitch, rangefinder distance) are selected at load time.
--
-- WHAT A DEAD SCRIPT DOES (read before trusting the fallback):
--   * MAIN1/MAIN3 fall back to SERVOn_TRIM (1500) only because this script calls
--     set_output_scaled() on functions 94/95. Without that call ArduPilot holds
--     the LAST PWM forever after the timeout expires. See send_outputs().
--   * Relays do NOT fall back. If the Lua VM stops, R1 and R2 stay in their last
--     state and the vehicle stays armed, so the engine and blade keep running. A
--     radio kill or failsafe then disarms but opens nothing. Only the key
--     covers that case.

-- ---------------------------------------------------------------------------
-- Configuration. Everything marked TUNE is a placeholder, not a measurement.
-- ---------------------------------------------------------------------------

-- Loop timing, for a required 50 Hz or better. The real rate runs below the nominal
-- one: SITL measured 78 Hz at 10 ms and only 53 Hz at 15 ms. Check it on the Pixhawk.
local LOOP_MS           = 10
-- How long one output command stays valid. Must comfortably exceed LOOP_MS.
local OUTPUT_TIMEOUT_MS = 150

-- Drive conversion. TUNE: scaling and positive direction are not yet measured.
local NEUTRAL_PWM     = 1500
local PWM_MIN         = 1000
local PWM_MAX         = 2000
local FORE_AFT_GAIN   = 1.0   -- TUNE
local DIFF_GAIN       = 1.0   -- TUNE
local FORE_AFT_SIGN   = -1    -- Field test 2026-11-05: +1 drove backward on a forward command
local DIFF_SIGN       = -1    -- Field test 2026-11-05: +1 turned CCW on a right command
-- L/R values outside this window are treated as invalid, so the output is neutral.
local INPUT_VALID_MIN = 800
local INPUT_VALID_MAX = 2200

-- Track relay timing. TUNE: controller boot time is not yet measured.
local BOOT_DELAY_MS        = 3000  -- neutral held after R1 closes, before motion
local DISARM_OPEN_DELAY_MS = 300   -- neutral held after disarm, before R1 opens

-- Tilt cutoff. TUNE: placeholder angle, not chosen for this machine or terrain.
local TILT = {}
TILT.LIMIT_DEG   = 30
TILT.DEBOUNCE_MS = 500
TILT.CLEAR_MS    = 2000  -- must be level and disarmed this long to clear

-- Optional forward rangefinder stop. Does nothing if no sensor is configured.
-- TUNE: grass and slopes are expected to cause false trips.
local RF = {}
RF.STOP_ENABLED    = true
RF.ORIENT          = 0     -- ROTATION_NONE, forward
RF.STOP_DISTANCE_M = 2.0
RF.DEBOUNCE_MS     = 300
RF.STATUS_OOR_LOW  = 2     -- RangeFinder::Status::OutOfRangeLow
RF.STATUS_GOOD     = 4     -- RangeFinder::Status::Good
-- In Manual, only trip on forward command so the operator can reverse away.
RF.MANUAL_FORWARD_DEADBAND = 20

-- Engine relay R2: COM to the kill wire, NC to ground, so de-energized kills the
-- engine. That is also the only blade stop (the clutch is manual). The policy, from
-- docs/wiring-and-code-map.md: energized after boot, then any disarm after arming,
-- failsafe, kill switch, tilt, rangefinder stop, or script fault drops it and latches
-- it off until the key is cycled, which reboots the Pixhawk.
-- TODO(user): leave false until the R2 wiring is confirmed on the mower. While false,
-- this script sends R2 no command from any path. With R2 wired and this false, the
-- engine stays killed, because nothing energizes the relay.
local R2_WIRING_CONFIRMED = false

-- Starter relay R3: NO contacts in parallel with the key's START contact. Cranks only
-- while the start button (CH5) is held, the vehicle is disarmed with R1 open, and R2 is
-- energized (engine not killed). Capped at STARTER.MAX_MS per press; the button must
-- be released and STARTER.REST_MS must pass before the next crank. The Pixhawk cannot
-- tell if the engine is running, so a press while running grinds the starter. Also
-- blocked while tilted past TILT.LIMIT_DEG, armed or not.
-- TODO(user): leave false until R3 is wired and confirmed not to close during boot.
-- While false, this script sends R3 no command.
local STARTER_WIRING_CONFIRMED = false
local STARTER = {}
STARTER.RC_CHAN  = 5     -- CT-6A in 5/6 mix: hold CH5 reads about 1000
STARTER.PRESS_PWM = 1200 -- at or below this (and valid) counts as held
STARTER.MAX_MS   = 2500
STARTER.REST_MS  = 5000

-- Radio loss, backup to ArduPilot's throttle failsafe. The SBUS-A never sets the SBUS
-- failsafe flag and freezes most channels on link loss. Only CH1/CH2 take the CT-6A
-- fail-safe values. CH2's fail-safe was set at full forward (995) with the throttle
-- stroke then cut to 85%, so live CH2 stays 1070-1920 and 995 only means link lost
-- (bench, 2026-11-05). Must match FS_THR_VALUE. CH2 below it, or no valid RC input,
-- counts as lost: disarm, kill the engine, block the starter.
local RADIO = {}
RADIO.LINK_CHAN    = 2
RADIO.LOST_BELOW   = 1030  -- FS_THR_VALUE
RADIO.LOST_MS      = 200

local GCS_MIN_INTERVAL_MS = 5000
local CONFIG_CHECK_MS     = 1000

-- ---------------------------------------------------------------------------
-- Hardware map. Must match mower.param.
-- ---------------------------------------------------------------------------
local CHAN_FORE_AFT   = 0   -- MAIN1, zero indexed
local CHAN_DIFF       = 2   -- MAIN3
local CHAN_LEFT       = 4   -- MAIN5, connected to nothing
local CHAN_RIGHT      = 5   -- MAIN6, connected to nothing
local FUNC_LEFT       = 73  -- Throttle Left
local FUNC_RIGHT      = 74  -- Throttle Right
local FUNC_SCRIPT1    = 94  -- Script Out 1
local FUNC_SCRIPT2    = 95  -- Script Out 2

local RELAY_TRACKS    = 0   -- R1, RELAY1
local RELAY_ENGINE    = 1   -- R2, RELAY2
local RELAY_STARTER   = 2   -- R3, RELAY3

local MODE = {}
MODE.MANUAL     = 0
MODE.HOLD       = 4
MODE.AUTO       = 10

local SEV = {}
SEV.CRITICAL    = 2
SEV.WARNING     = 4
SEV.INFO        = 6

-- States of the arm sequence.
local ST_DISARMED  = 0  -- R1 open, neutral
local ST_BOOTING   = 1  -- armed, R1 closed, neutral, waiting for controllers
local ST_RUN       = 2  -- armed, motion allowed
local ST_DISARMING = 3  -- disarmed, neutral, R1 still closed briefly

local state        = ST_DISARMED
local state_ms     = 0
local fault_msg    = nil  -- latched until reboot
local tilt_latched = false
local tilt_over_ms = nil
local tilt_ok_ms   = nil
local is_level     = false  -- within TILT.LIMIT_DEG on the last check
local rf_near_ms   = nil
local rf_was_seen  = false
local engine_killed = false  -- latched until reboot
local next_cfg_ms  = 0
local last_msg_ms  = {}
local crank_start_ms   = nil    -- set while cranking
local crank_end_ms     = nil    -- when the last crank stopped
local starter_released = false  -- button must be seen released before any crank
local radio_bad_ms     = nil
local radio_lost       = true   -- until a linked CH4 is seen

-- Rover 4.7+ adds the _rad names and deprecates the old ones. 4.6 only has the old.
local get_roll  = ahrs.get_roll_rad  or ahrs.get_roll
local get_pitch = ahrs.get_pitch_rad or ahrs.get_pitch

local function now_ms()
    return millis():toint()
end

local function say(key, severity, text)
    local now = now_ms()
    local last = last_msg_ms[key]
    if last == nil or (now - last) >= GCS_MIN_INTERVAL_MS then
        last_msg_ms[key] = now
        gcs:send_text(severity, "Mower: " .. text)
    end
end

local function clamp_pwm(value)
    return math.floor(math.max(PWM_MIN, math.min(PWM_MAX, value)) + 0.5)
end

-- Sends one output pair. Called every loop, in every state.
local function send_outputs(fore_aft, differential)
    -- REQUIRED, do not remove. set_output_pwm_chan_timeout() only falls back to
    -- the scaled value if the channel's "have PWM" bit is clear. Scaled 0 on a
    -- Script Out function is SERVOn_TRIM. Without this call the channel holds its
    -- last PWM after the timeout. Checked in libraries/SRV_Channel and in SITL.
    SRV_Channels:set_output_scaled(FUNC_SCRIPT1, 0)
    SRV_Channels:set_output_scaled(FUNC_SCRIPT2, 0)
    SRV_Channels:set_output_pwm_chan_timeout(CHAN_FORE_AFT, fore_aft, OUTPUT_TIMEOUT_MS)
    SRV_Channels:set_output_pwm_chan_timeout(CHAN_DIFF, differential, OUTPUT_TIMEOUT_MS)
end

local function send_neutral()
    send_outputs(NEUTRAL_PWM, NEUTRAL_PWM)
end

local function valid_input(pwm)
    return type(pwm) == "number" and pwm >= INPUT_VALID_MIN and pwm <= INPUT_VALID_MAX
end

-- Returns the left and right commands. If either is invalid both are neutral,
-- because one neutral track and one driven track would still be a turn.
local function read_left_right()
    local left = SRV_Channels:get_output_pwm(FUNC_LEFT)
    local right = SRV_Channels:get_output_pwm(FUNC_RIGHT)
    if valid_input(left) and valid_input(right) then
        return left, right, true
    end
    return NEUTRAL_PWM, NEUTRAL_PWM, false
end

-- The KEYIUAV box locks up (tracks twitch, no motion) when one track is asked to reverse
-- while the other drives forward with fore-aft non-zero. It pivots cleanly when fore-aft is
-- neutral. So any counter-rotating request becomes a pure pivot (field test 2026-11-05).
local STEER = {}
STEER.PIVOT_DEADBAND = 20  -- us; smaller per-track commands count as stopped

-- The box ignores small differential commands, so small steering corrections on straights
-- did nothing until they built up and lanes drifted. While driving (fore-aft beyond
-- STEER.COMP_MIN_FORE), a differential beyond STEER.COMP_MIN_DIFF is lifted past the dead
-- zone, scaled so full command stays full. TUNE STEER_DEADZONE_US by measurement.
local STEER_DEADZONE_US  = 25
STEER.COMP_MIN_FORE = 50
STEER.COMP_MIN_DIFF = 10

local function convert(left, right)
    local l = left - NEUTRAL_PWM
    local r = right - NEUTRAL_PWM
    if (l > STEER.PIVOT_DEADBAND and r < -STEER.PIVOT_DEADBAND) or (l < -STEER.PIVOT_DEADBAND and r > STEER.PIVOT_DEADBAND) then
        local d = (l - r) / 2
        l, r = d, -d
    end
    local fa = (l + r) / 2
    local df = (l - r) / 2
    if math.abs(fa) > STEER.COMP_MIN_FORE and math.abs(df) > STEER.COMP_MIN_DIFF then
        local mag = STEER_DEADZONE_US + math.abs(df) * (500 - STEER_DEADZONE_US) / 500
        df = (df > 0) and mag or -mag
    end
    local fore_aft = NEUTRAL_PWM + FORE_AFT_SIGN * FORE_AFT_GAIN * fa
    local differential = NEUTRAL_PWM + DIFF_SIGN * DIFF_GAIN * df
    return clamp_pwm(fore_aft), clamp_pwm(differential)
end

-- Drops R2 and latches it off until reboot. Does nothing until the wiring is confirmed.
local function kill_engine()
    if not R2_WIRING_CONFIRMED then
        return
    end
    engine_killed = true
    relay:off(RELAY_ENGINE)
    say("engine", SEV.CRITICAL, "engine killed, cycle key")
end

local function starter_off()
    if not STARTER_WIRING_CONFIRMED then
        return
    end
    relay:off(RELAY_STARTER)
end

-- Runs each step separately so one failure cannot skip the others.
local function emergency_outputs()
    pcall(starter_off)
    pcall(send_neutral)
    pcall(relay.off, relay, RELAY_TRACKS)
    pcall(kill_engine)
    pcall(function()
        if arming:is_armed() then
            arming:disarm()
        end
    end)
end

local function latch_fault(text)
    if fault_msg == nil then
        fault_msg = text
    end
    emergency_outputs()
end

-- Any mismatch means the fallback or the track cut cannot be trusted.
local function config_problem()
    if not relay:enabled(RELAY_TRACKS) then
        return "RELAY1 not configured"
    end
    if R2_WIRING_CONFIRMED and not relay:enabled(RELAY_ENGINE) then
        return "RELAY2 not configured"
    end
    if STARTER_WIRING_CONFIRMED and not R2_WIRING_CONFIRMED then
        return "starter needs R2 confirmed"
    end
    if STARTER_WIRING_CONFIRMED and not relay:enabled(RELAY_STARTER) then
        return "RELAY3 not configured"
    end
    if SRV_Channels:find_channel(FUNC_SCRIPT1) ~= CHAN_FORE_AFT
        or SRV_Channels:find_channel(FUNC_SCRIPT2) ~= CHAN_DIFF then
        return "SERVO1/3 must be func 94/95"
    end
    if SRV_Channels:find_channel(FUNC_LEFT) ~= CHAN_LEFT
        or SRV_Channels:find_channel(FUNC_RIGHT) ~= CHAN_RIGHT then
        return "SERVO5/6 must be func 73/74"
    end
    for _, name in ipairs({"SERVO1_TRIM", "SERVO3_TRIM", "SERVO5_TRIM", "SERVO6_TRIM"}) do
        if param:get(name) ~= NEUTRAL_PWM then
            return name .. " must be 1500"
        end
    end
    return nil
end

local function update_tilt(now, armed)
    local roll = math.deg(get_roll(ahrs))
    local pitch = math.deg(get_pitch(ahrs))
    -- A NaN fails this comparison, so it counts as over the limit.
    local within = math.abs(roll) <= TILT.LIMIT_DEG and math.abs(pitch) <= TILT.LIMIT_DEG
    is_level = within

    if armed and not within then
        tilt_over_ms = tilt_over_ms or now
        if not tilt_latched and (now - tilt_over_ms) >= TILT.DEBOUNCE_MS then
            tilt_latched = true
            say("tilt", SEV.CRITICAL, "TILT cutoff, disarming")
        end
    else
        tilt_over_ms = nil
    end

    if tilt_latched and not armed and within then
        tilt_ok_ms = tilt_ok_ms or now
        if (now - tilt_ok_ms) >= TILT.CLEAR_MS then
            tilt_latched = false
            tilt_ok_ms = nil
            say("tilt_clear", SEV.INFO, "tilt cleared")
        end
    else
        tilt_ok_ms = nil
    end
end

-- Returns true if a configured, working sensor sees something inside the limit.
local function rangefinder_near()
    if not RF.STOP_ENABLED or rangefinder == nil then
        return false
    end
    if not rangefinder:has_orientation(RF.ORIENT) then
        return false
    end
    local status = rangefinder:status_orient(RF.ORIENT)
    if status == RF.STATUS_OOR_LOW then
        -- Closer than the sensor's minimum range is still something close.
        rf_was_seen = true
        return true
    end
    if status ~= RF.STATUS_GOOD then
        if rf_was_seen and status < RF.STATUS_OOR_LOW then
            say("rf_lost", SEV.WARNING, "rangefinder lost, no stop")
        end
        return false
    end
    rf_was_seen = true
    local distance_m
    if rangefinder.distance_orient then
        distance_m = rangefinder:distance_orient(RF.ORIENT)  -- 4.7 and later
    else
        distance_m = rangefinder:distance_cm_orient(RF.ORIENT) / 100
    end
    return distance_m < RF.STOP_DISTANCE_M
end

local function update_rangefinder(now, mode, left, right)
    local near = rangefinder_near()
    if not near or state ~= ST_RUN or mode == MODE.HOLD then
        rf_near_ms = nil
        return
    end
    if mode == MODE.MANUAL then
        local forward = ((left - NEUTRAL_PWM) + (right - NEUTRAL_PWM)) / 2
        if forward <= RF.MANUAL_FORWARD_DEADBAND then
            rf_near_ms = nil
            return
        end
    end
    rf_near_ms = rf_near_ms or now
    if (now - rf_near_ms) >= RF.DEBOUNCE_MS then
        rf_near_ms = nil
        kill_engine()
        say("rf", SEV.WARNING, "obstacle stop, Hold")
        if not vehicle:set_mode(MODE.HOLD) then
            say("rf_hold", SEV.CRITICAL, "Hold refused, disarming")
            arming:disarm()
        end
    end
end

local function update_engine_relay()
    if not R2_WIRING_CONFIRMED then
        return
    end
    if engine_killed then
        relay:off(RELAY_ENGINE)
    else
        relay:on(RELAY_ENGINE)
    end
end

local function update_radio(now, armed)
    local ok = false
    if rc:has_valid_input() then
        local pwm = rc:get_pwm(RADIO.LINK_CHAN)
        ok = pwm >= RADIO.LOST_BELOW and pwm <= INPUT_VALID_MAX
    end
    if ok then
        radio_bad_ms = nil
        if radio_lost then
            radio_lost = false
            say("radio_ok", SEV.INFO, "radio linked")
        end
        return
    end
    radio_bad_ms = radio_bad_ms or now
    if not radio_lost and (now - radio_bad_ms) >= RADIO.LOST_MS then
        radio_lost = true
        say("radio_lost", SEV.CRITICAL, "radio lost")
    end
    -- Matches FS_THR_ENABLE 2: a mission in Auto keeps going without the transmitter.
    -- Every other mode disarms, because the CT-6A's loss value on CH2 is full forward.
    if radio_lost and armed and vehicle:get_mode() ~= MODE.AUTO then
        say("radio_disarm", SEV.CRITICAL, "radio lost, disarming")
        arming:disarm()
    elseif radio_lost and armed then
        say("radio_auto", SEV.WARNING, "radio lost, Auto continues")
    end
end

-- Must run after update_track_relay, so state reflects this cycle's arm state.
local function update_starter(now, armed)
    if not STARTER_WIRING_CONFIRMED then
        return
    end
    local pwm = rc:has_valid_input() and rc:get_pwm(STARTER.RC_CHAN) or 0
    local held = pwm >= INPUT_VALID_MIN and pwm <= STARTER.PRESS_PWM
    if not held then
        starter_released = true
    end
    local allowed = held and starter_released and not armed and state == ST_DISARMED
        and not engine_killed and is_level and not radio_lost
    if crank_start_ms ~= nil then
        if not allowed or (now - crank_start_ms) >= STARTER.MAX_MS then
            if allowed then
                say("crank_max", SEV.WARNING, "crank limit, release button")
            end
            crank_start_ms = nil
            crank_end_ms = now
            starter_released = false
        end
    elseif allowed and (crank_end_ms == nil or (now - crank_end_ms) >= STARTER.REST_MS) then
        crank_start_ms = now
        say("crank", SEV.INFO, "cranking")
    end
    if crank_start_ms ~= nil then
        relay:on(RELAY_STARTER)
    else
        relay:off(RELAY_STARTER)
    end
end

local function set_state(new_state, now)
    state = new_state
    state_ms = now
end

local function update_track_relay(now, armed)
    if armed then
        if state == ST_DISARMED or state == ST_DISARMING then
            -- Also covers a script restart while armed: wait again, neutral first.
            set_state(ST_BOOTING, now)
        end
        relay:on(RELAY_TRACKS)
        if state == ST_BOOTING and (now - state_ms) >= BOOT_DELAY_MS then
            set_state(ST_RUN, now)
        end
    else
        if state == ST_BOOTING or state == ST_RUN then
            set_state(ST_DISARMING, now)
            kill_engine()  -- any disarm after arming, per the R2 policy
        end
        if state == ST_DISARMING and (now - state_ms) >= DISARM_OPEN_DELAY_MS then
            set_state(ST_DISARMED, now)
        end
        if state == ST_DISARMED then
            relay:off(RELAY_TRACKS)
        end
    end
end

local function step()
    local now = now_ms()
    local armed = arming:is_armed()

    if fault_msg == nil and (now - next_cfg_ms) >= 0 then
        next_cfg_ms = now + CONFIG_CHECK_MS
        local problem = config_problem()
        if problem ~= nil then
            fault_msg = problem
        end
    end

    update_tilt(now, armed)
    update_radio(now, armed)
    armed = arming:is_armed()

    if fault_msg ~= nil or tilt_latched then
        set_state(ST_DISARMED, now)
        emergency_outputs()
        if fault_msg ~= nil then
            say("fault", SEV.CRITICAL, "FAULT " .. fault_msg)
        end
        return
    end

    update_track_relay(now, armed)
    update_starter(now, armed)

    local mode = vehicle:get_mode()
    local left, right, inputs_ok = read_left_right()
    if armed and not inputs_ok then
        say("input", SEV.WARNING, "invalid L/R, neutral")
    end

    -- These run first so a stop takes effect on this same cycle.
    update_rangefinder(now, mode, left, right)
    update_engine_relay()

    -- Re-read the mode: a rangefinder stop above may have just changed it. Hold is
    -- forced to neutral here because Rover ramps L/R down at MOT_SLEWRATE (about a
    -- second from full at the default), which would delay the stop.
    local stop_now = vehicle:get_mode() == MODE.HOLD or SRV_Channels:get_emergency_stop()
    if state == ST_RUN and armed and inputs_ok and not stop_now then
        send_outputs(convert(left, right))
    else
        send_neutral()
    end
end

local function loop()
    local ok, err = pcall(step)
    if not ok then
        latch_fault("Lua error")
        pcall(say, "lua_err", SEV.CRITICAL, "ERROR " .. tostring(err))
    end
    return loop, LOOP_MS
end

-- Start neutral and with the relay open, before the first scheduled loop.
local start_ok = pcall(function()
    SRV_Channels:set_angle(FUNC_SCRIPT1, 4500)  -- makes scaled 0 mean SERVOn_TRIM
    SRV_Channels:set_angle(FUNC_SCRIPT2, 4500)
    starter_off()
    send_neutral()
    relay:off(RELAY_TRACKS)
end)
if not start_ok then
    latch_fault("start failed")
end
gcs:send_text(SEV.INFO, "Mower: safety script started (draft)")

return loop, LOOP_MS
