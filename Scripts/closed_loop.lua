-- closed_loop.lua
-- ArduCopter Lua script: arm, take off, fly a closed square route,
-- return above the launch point, and land.
--
-- SAFETY / ACTIVATION:
--   Assign one spare RC channel to Scripting1, e.g. RC7_OPTION = 300.
--   The switch must be moved LOW after boot, then HIGH to start.
--   Moving it LOW during flight commands LAND.
--
-- Intended for SITL validation before use on a physical aircraft.

local TAKEOFF_ALT_M       = 3.0
local ALLOW_AUTO_ARM       = false -- keep false for physical flight; pilot arms manually
local LEG_LENGTH_M        = 5.0
local ARRIVAL_RADIUS_M    = 0.8
local UPDATE_PERIOD_MS    = 200
local ARM_SETTLE_MS       = 1500
local TAKEOFF_TIMEOUT_MS  = 20000
local LEG_TIMEOUT_MS      = 30000

local MODE_GUIDED = 4
local MODE_LAND   = 9

local AUX_SCRIPTING1 = 300
local AUX_LOW        = 0
local AUX_HIGH       = 2

local STATE_WAIT          = 0
local STATE_ARM_SETTLE    = 1
local STATE_TAKEOFF       = 2
local STATE_NAVIGATE      = 3
local STATE_LANDING       = 4
local STATE_PILOT_CONTROL = 5
local STATE_DONE          = 6

local state = STATE_WAIT
local state_started_ms = 0
local switch_seen_low = false
local previous_aux_position = nil
local targets = {}
local target_index = 0

local function now_ms()
    return millis():tofloat()
end

local function send_message(text)
    gcs:send_text(6, "LOOP: " .. text)
end

local function enter_state(new_state)
    state = new_state
    state_started_ms = now_ms()
end

local function state_timed_out(timeout_ms)
    return (now_ms() - state_started_ms) > timeout_ms
end

local function make_target(origin, north_m, east_m)
    local target = origin:copy()
    target:offset(north_m, east_m)

    -- ArduCopter 4.6.x Location API stores altitude in centimetres.
    -- Convert the copied location to ABOVE_HOME frame, then set 3 m = 300 cm.
    if not target:change_alt_frame(1) then
        return nil
    end
    target:alt(math.floor(TAKEOFF_ALT_M * 100.0 + 0.5))
    return target
end

local function build_closed_square(origin)
    local square = {
        make_target(origin, LEG_LENGTH_M, 0.0),
        make_target(origin, LEG_LENGTH_M, LEG_LENGTH_M),
        make_target(origin, 0.0, LEG_LENGTH_M),
        make_target(origin, 0.0, 0.0)
    }

    for i = 1, #square do
        if square[i] == nil then
            return nil
        end
    end
    return square
end

local function command_land(reason)
    send_message(reason .. "; switching to LAND")
    if arming:is_armed() then
        vehicle:set_mode(MODE_LAND)
    end
    enter_state(STATE_LANDING)
end

local function command_target(index)
    local target = targets[index]
    if target == nil then
        command_land("invalid target index")
        return false
    end

    if not vehicle:set_target_location(target) then
        command_land("failed to set target " .. tostring(index))
        return false
    end

    target_index = index
    enter_state(STATE_NAVIGATE)
    send_message("flying to target " .. tostring(index) .. "/" .. tostring(#targets))
    return true
end

local function start_sequence()
    if not ahrs:healthy() then
        send_message("start rejected: AHRS unhealthy")
        return
    end

    if not ahrs:home_is_set() then
        send_message("start rejected: home not set")
        return
    end

    if not arming:is_armed() and not arming:pre_arm_checks() then
        send_message("start rejected: pre-arm checks failed")
        return
    end

    local launch_location = ahrs:get_location()
    if launch_location == nil then
        send_message("start rejected: no valid position estimate")
        return
    end

    targets = build_closed_square(launch_location)
    if targets == nil then
        send_message("start rejected: could not create ABOVE_HOME targets")
        return
    end

    if not vehicle:set_mode(MODE_GUIDED) then
        send_message("start rejected: could not enter GUIDED")
        return
    end

    if not arming:is_armed() then
        if not ALLOW_AUTO_ARM then
            send_message("start rejected: arm manually, then move switch LOW-HIGH")
            return
        end
        if not arming:arm() then
            send_message("start rejected: automatic arming failed")
            return
        end
    end

    enter_state(STATE_ARM_SETTLE)
    send_message("armed in GUIDED; preparing takeoff")
end

local function check_pilot_override()
    if arming:is_armed() and vehicle:get_mode() ~= MODE_GUIDED then
        send_message("mode changed by pilot/failsafe; script relinquishing control")
        enter_state(STATE_PILOT_CONTROL)
        return true
    end
    return false
end

function update()
    local aux_position = rc:get_aux_cached(AUX_SCRIPTING1)

    if aux_position == AUX_LOW then
        switch_seen_low = true
    end

    local start_edge = switch_seen_low
        and aux_position == AUX_HIGH
        and previous_aux_position ~= AUX_HIGH

    previous_aux_position = aux_position

    -- LOW is the in-flight abort command.
    if state >= STATE_ARM_SETTLE
        and state <= STATE_NAVIGATE
        and aux_position == AUX_LOW then
        command_land("abort switch selected")
        return update, UPDATE_PERIOD_MS
    end

    if state == STATE_WAIT then
        if start_edge then
            start_sequence()
        end

    elseif state == STATE_ARM_SETTLE then
        if check_pilot_override() then
            return update, UPDATE_PERIOD_MS
        end

        if (now_ms() - state_started_ms) >= ARM_SETTLE_MS then
            if vehicle:start_takeoff(TAKEOFF_ALT_M) then
                enter_state(STATE_TAKEOFF)
                send_message("takeoff commanded to " .. tostring(TAKEOFF_ALT_M) .. " m")
            elseif state_timed_out(5000) then
                command_land("takeoff command failed")
            end
        end

    elseif state == STATE_TAKEOFF then
        if check_pilot_override() then
            return update, UPDATE_PERIOD_MS
        end

        local down_from_home_m = ahrs:get_relative_position_D_home()
        if down_from_home_m ~= nil then
            local altitude_above_home_m = -down_from_home_m
            if altitude_above_home_m >= (TAKEOFF_ALT_M - 0.5) then
                command_target(1)
            elseif state_timed_out(TAKEOFF_TIMEOUT_MS) then
                command_land("takeoff timeout")
            end
        elseif state_timed_out(TAKEOFF_TIMEOUT_MS) then
            command_land("no altitude estimate during takeoff")
        end

    elseif state == STATE_NAVIGATE then
        if check_pilot_override() then
            return update, UPDATE_PERIOD_MS
        end

        local current_location = ahrs:get_location()
        local current_target = targets[target_index]

        if current_location == nil or current_target == nil then
            command_land("position/target unavailable")
        else
            local distance_m = current_location:get_distance(current_target)
            if distance_m <= ARRIVAL_RADIUS_M then
                send_message("target " .. tostring(target_index)
                    .. " reached; horizontal error="
                    .. string.format("%.2f", distance_m) .. " m")

                local next_index = target_index + 1
                if next_index <= #targets then
                    command_target(next_index)
                else
                    command_land("closed route complete; return error="
                        .. string.format("%.2f", distance_m) .. " m")
                end
            elseif state_timed_out(LEG_TIMEOUT_MS) then
                command_land("target " .. tostring(target_index) .. " timeout")
            end
        end

    elseif state == STATE_LANDING then
        if not arming:is_armed() then
            enter_state(STATE_DONE)
            send_message("landed and disarmed")
        elseif vehicle:get_mode() ~= MODE_LAND then
            send_message("landing mode changed; script relinquishing control")
            enter_state(STATE_PILOT_CONTROL)
        end

    elseif state == STATE_PILOT_CONTROL then
        -- Deliberately do nothing: pilot or failsafe owns the aircraft.
        if not arming:is_armed() then
            enter_state(STATE_DONE)
            send_message("vehicle disarmed after pilot/failsafe takeover")
        end

    elseif state == STATE_DONE then
        -- Require the switch to return LOW before another run can be started.
        if not arming:is_armed() and aux_position == AUX_LOW then
            targets = {}
            target_index = 0
            enter_state(STATE_WAIT)
            send_message("reset; ready for another low-to-high start")
        end
    end

    return update, UPDATE_PERIOD_MS
end

send_message("script loaded; move Scripting1 LOW, then HIGH to start")
return update, UPDATE_PERIOD_MS
