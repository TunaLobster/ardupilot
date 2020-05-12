-- This script will preform a control surface doublet
-- Charlie Johnson, OSU

local DOUBLET_TIME = 500 -- period of doublet signal in ms
local DOUBLET_CHANNEL = 6 -- channel to receive HIGH PWM from (>1700)
local DOUBLET_FUCNTION = 19 -- which control surface SERVOx_FUNCTION number will have a doublet happen
-- A (Servo 1, Function 4), E (Servo 2, Function 19), and R (Servo 4, Function 21)

-- flight mode numbers for plane
local MODE_MANUAL = 0
local MODE_CIRCLE = 1
local MODE_CRUISE = 7
-- local MODE_AUTO = 10
local MODE_RTL = 11

-- callback interval should be frequent enough that a consistent doublet can be produced
-- but not fast enougth to interfer with the main loop
local callback_time = math.floor(DOUBLET_TIME / 10)
-- this will work in the FUTURE and might not be necessary
-- local loop_period = math.ceil(1/param:get('SCHED_LOOP_RATE'))
-- if callback_time < loop_period*1.1 then
--     callback_time = math.floor(loop_period*1.1)
-- end

-- store the info between callbacks
-- set at the start of each doublet
local start_time = -1
local end_time = -1
local start_mode = -1
local now = -1

-- test if getting the servo min and max pwm from parameters is possible
-- it will be in the future. Plane 4.0.5 doesn't have this yet.
local doublet_srv_chan = SRV_Channels:find_channel(DOUBLET_FUCNTION)
local doublet_srv_min = param:get("SERVO" .. doublet_srv_chan .. "_MIN")
local doublet_srv_max = param:get("SERVO" .. doublet_srv_chan .. "_MAX")
local doublet_srv_trim = param:get("SERVO" .. doublet_srv_chan .. "_TRIM")
-- local doublet_srv_min = 1100
-- local doublet_srv_max = 1900
-- local doublet_srv_trim = 1580

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
    if arming:is_armed() == false and (rc:get_pwm(DOUBLET_CHANNEL) > 1700 or start_time ~= -1) then
        -- check if aircraft is entering a dangerous flight condition FIXME
        -- local current_hagl = ahrs:get_hagl()
        -- if current_hagl and current_hagl < 20 then
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
            start_mode = vehicle:get_mode()
            -- set the desired control surface servo ouput to SCRIPT1 function
            param:set("SERVO" .. doublet_srv_chan .. "_FUNCTION", 94)
            -- notify the gcs that we are starting a manuver
            gcs:send_text(6, "STARTING DOUBLET")
            -- enter manual mode
            retry_set_mode(MODE_MANUAL)
        -- vehicle:set_mode(MODE_MANUAL)
        end
        -- split time evenly between high and low signal
        if now < start_time + (DOUBLET_TIME / 2) then
            -- servo.set_output_pwm(94, doublet_srv_max)
            -- gcs:send_text(6, "doublet high")
            SRV_Channels:set_output_pwm(94, doublet_srv_max)
        elseif now < start_time + DOUBLET_TIME then
            -- servo.set_output_pwm(94, doublet_srv_min)
            -- gcs:send_text(6, "doublet low")
            SRV_Channels:set_output_pwm(94, doublet_srv_min)
        elseif now < start_time + (DOUBLET_TIME * 2) then
            -- servo.set_output_pwm(94, doublet_srv_trim)
            -- gcs:send_text(6, "doublet neutral")
            -- done with the doublet, wait for the dynamics to stop
            SRV_Channels:set_output_pwm(94, doublet_srv_trim)
        elseif now > start_time + (DOUBLET_TIME * 2) and end_time == -1 then
            -- elseif now > start_time + (DOUBLET_TIME * 2) and end_time ~= -1 then
            -- return to a safer mode. In this case I picked CIRCLE
            -- return to the original flight mode (FUTURE feature)
            -- retry_set_mode(MODE_CIRCLE)
            -- vehicle:set_mode(MODE_CIRCLE)
            end_time = now
            gcs:send_text(6, "DOUBLET FINISHED")
        elseif end_time ~= -1 then
            -- wait for RC input channel to go low
            gcs:send_text(6, "RC" .. DOUBLET_CHANNEL .. " still high")
        else
            -- this should not be reached
            gcs:send_text(6, "this should not be reached")
        end
    elseif now ~= -1 and end_time ~= -1 then
        gcs:send_text(6, "RETURN TO PREVIOUS FLIGHT MODE")
        now = -1
        end_time = -1
        retry_set_mode(MODE_CRUISE)
    else
        -- gcs:send_text(6, 'normal operation_'..start_time)
        -- do this each time to make sure that plane is in a safe mode with proper settings
        param:set("SERVO" .. doublet_srv_chan .. "_FUNCTION", DOUBLET_FUCNTION)
        start_time = -1
    end
    return doublet, callback_time
end

gcs:send_text(6, "doublet.lua is running")
return doublet(), callback_time
