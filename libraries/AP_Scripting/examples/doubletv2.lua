-- Doublet version 2

-- This script will preform a control surface doublet
-- Charlie Johnson, OSU

local DOUBLET_TIME = 2000 -- period of doublet signal in ms
local DOUBLET_ACTION_CHANNEL = 6 -- RCIN channel to start a doublet when high (>1700)
local DOUBLET_CHOICE_CHANNEL = 7 -- RCIN channel to choose elevator (low) or rudder (high)
local DOUBLET_FUCNTION = 19 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
-- A (Servo 1, Function 4), E (Servo 2, Function 19), and R (Servo 4, Function 21)

-- flight mode numbers for plane
local MODE_MANUAL = 0
local MODE_CIRCLE = 1
local MODE_STABILIZE = 2
local MODE_CRUISE = 7
-- local MODE_AUTO = 10
local MODE_RTL = 11
local K_AILERON = 4
local K_ELEVATOR = 19
local K_THROTTLE = 70
local K_RUDDER = 21

-- callback interval should be frequent enough that a consistent doublet can be produced
local callback_time = math.floor(DOUBLET_TIME / 10)

-- store the info between callbacks
-- set at the start of each doublet
local start_time = -1
local end_time = -1
local now = -1

-- store information about the vehicle
local doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
local doublet_srv_min = param:get("SERVO" .. doublet_srv_chan+1 .. "_MIN")
local doublet_srv_max = param:get("SERVO" .. doublet_srv_chan+1 .. "_MAX")
local doublet_srv_trim = param:get("SERVO" .. doublet_srv_chan+1 .. "_TRIM")
local pre_doublet_mode = vehicle:get_mode()
local current_hagl = ahrs:get_hagl()

function retry_set_mode(mode)
    if vehicle:set_mode(mode) then
        -- if the mode was set successfully, carry on as normal
        return doublet, 1
    else
        -- if the mode was not set successfully, try again ASAP
        return retry_set_mode, 1
    end
end

function doublet()
    callback_time = DOUBLET_TIME/10
    -- if arming:is_armed() == true and (rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 or start_time ~= -1) then
    if (rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 or start_time ~= -1) then
        -- check if aircraft is entering a dangerous flight condition FIXME
        -- this is like a cheap geofence floor
        -- current_hagl = ahrs:get_hagl()
        -- if current_hagl < 20 then
        --     retry_set_mode(MODE_RTL)
        -- end

        -- start a quick doublet based on some math/logic
        -- can't sit here and do the whole thing
        -- the autopilot will kill this if we stay here
        now = millis()
        if start_time == -1 then
            -- this is the time that we started
            -- stop time will be start_time + DOUBLET_TIME
            start_time = now
            end_time = -1
            -- notify the gcs that we are starting a doublet
            gcs:send_text(6, "STARTING DOUBLET")
            -- are we doing a doublet on elevator or rudder? set the other controls to trim
            local doublet_choice_pwm = rc:get_pwm(DOUBLET_CHOICE_CHANNEL)
            local trim_funcs = {}
            if doublet_choice_pwm < 1500 then
                -- doublet on elevator
                DOUBLET_FUCNTION = K_ELEVATOR
                trim_funcs = {K_AILERON, K_RUDDER}
            else
                -- doublet on rudder
                DOUBLET_FUCNTION = K_RUDDER
                trim_funcs = {K_ELEVATOR, K_AILERON}
            end
            -- get info on the doublet channel
            doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
            doublet_srv_min = param:get("SERVO" .. doublet_srv_chan+1 .. "_MIN")
            doublet_srv_max = param:get("SERVO" .. doublet_srv_chan+1 .. "_MAX")
            doublet_srv_trim = param:get("SERVO" .. doublet_srv_chan+1 .. "_TRIM")
            pre_doublet_mode = vehicle:get_mode()
            -- set the channels that need to be still to trim until the doublet is done
            for i=1, 2 do
                local trim_chan = SRV_Channels:find_channel(trim_funcs[i])
                SRV_Channels:set_output_pwm_chan_timeout(trim_chan, param:get("SERVO" .. trim_chan+1 .. "_TRIM"), DOUBLET_TIME*2)
            end
            -- get the current throttle PWM and pin it there until the doublet is done
            local pre_doublet_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
            SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_THROTTLE),pre_doublet_throttle,DOUBLET_TIME*2)
            -- enter manual mode
            retry_set_mode(MODE_MANUAL)
        end
        -- split time evenly between high and low signal
        if now < start_time + (DOUBLET_TIME / 2) then
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, doublet_srv_max, DOUBLET_TIME/2+100)
        elseif now < start_time + DOUBLET_TIME then
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, doublet_srv_min, DOUBLET_TIME/2+100)
        elseif now < start_time + (DOUBLET_TIME * 2) then
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, doublet_srv_trim, DOUBLET_TIME)
        elseif now > start_time + (DOUBLET_TIME * 2) and end_time == -1 then
            -- release the control surface to the autopilot
            -- return to normal flight
            SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan, doublet_srv_trim, 0)
            end_time = now
            gcs:send_text(6, "DOUBLET FINISHED")
            retry_set_mode(pre_doublet_mode)
        elseif end_time ~= -1 and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 then
            -- wait for RC input channel to go low
            gcs:send_text(6, "RC" .. DOUBLET_ACTION_CHANNEL .. " still high")
            callback_time = 500
            start_time = -1
        else
            gcs:send_text(6, "this should not be reached")
        end
    elseif now ~= -1 and end_time ~= -1 then
        gcs:send_text(6, "RETURN TO PREVIOUS FLIGHT MODE")
        now = -1
        end_time = -1
        -- clear all of the timeout
        control_functions = {K_AILERON, K_ELEVATOR,K_THROTTLE, K_RUDDER}
        for i=1, 4 do
            local control_chan = SRV_Channels:find_channel(control_functions[i])
            SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan+1 .. "_TRIM"), 0)
        end
        retry_set_mode(pre_doublet_mode)
    else
        -- gcs:send_text(6, 'normal operation_'..start_time)
        -- do this each time to make sure that plane is in a safe mode with proper settings
        -- param:set("SERVO" .. doublet_srv_chan .. "_FUNCTION", DOUBLET_FUCNTION)
        start_time = -1
    end
    return doublet, callback_time
end

gcs:send_text(6, "doublet.lua is running")
return doublet(), callback_time
