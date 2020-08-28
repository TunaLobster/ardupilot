clc
clearvars
close all
addpath(genpath('../../MATLAB'))

% Validate ArduPilot code with simple motions

% Set constants
state.gravity_mss = 9.80665; % (m/s^2)
state.enable_print = 1;


% Setup the time step size
max_timestep = 1/50;
state.physics_time = 0;

% define init and time setup functions
init_function = @init;
physics_function = @physics_step;

% setup connection
SITL_connector(state,init_function,physics_function,max_timestep);

% Simulator model must take and return a structure with the felids: 
% gyro(roll, pitch, yaw) (radians/sec) body frame
% attitude(roll, pitch yaw) (radians)
% accel(north, east, down) (m/s^2) body frame
% velocity(north, east,down) (m/s) earth frame
% position(north, east, down) (m) earth frame 
% the structure can have any other felids required for the physics model

% init values
function state = init(state)
    state.gyro = [0;0;0]; % (rad/sec)
    state.dcm = diag([1,1,1]); % direction cosine matrix
    state.attitude = [0;0;0]; % (radians) (roll, pitch, yaw)
    state.accel = [0;0;0]; % (m/s^2) body frame
    state.velocity = [0;0;0]; % (m/s) earth frame
    state.position = [0;0;0]; % (m) earth frame
    state.bf_velo = [0;0;0]; % (m/s) body frame
end

function rate = angle2rate(state,axis,desired_attitude)
    rate = (deg2rad(desired_attitude) - state.attitude(axis)) / state.delta_t;
end

function state = axis_angles(state,window_time,max_angle,max_attitude_rate)
    if state.physics_time < window_time
        if state.enable_print == 1
            state.enable_print = 0;
            fprintf('start motion\n')
        end
        % Roll 15 deg at max_attitude_rate
        state.gyro = [angle2rate(state,1,max_angle);0;0];
    elseif state.physics_time < window_time * 2
        state.enable_print = 1;
        state.gyro = [angle2rate(state,1,0);0;0];
    elseif state.physics_time < window_time * 3
        % Pitch 5 deg
        state.gyro = [0;angle2rate(state,2,5);0];
    elseif state.physics_time < window_time * 4
        state.gyro = [0;angle2rate(state,2,0);0];
    elseif state.physics_time < window_time * 5
        % Yaw 15 deg
        state.gyro = [0;0;angle2rate(state,3,max_angle)];
    elseif state.physics_time < window_time * 6
        state.gyro = [0;0;angle2rate(state,3,0)];
    end
    state.gyro = max(state.gyro,deg2rad(-abs(max_attitude_rate)));
    state.gyro = min(state.gyro,deg2rad(abs(max_attitude_rate)));
end

function state = axis_rates(state,window_time)
    if state.physics_time < window_time * 7
        % Roll 5 deg/sec
        state.gyro = [deg2rad(5);0;0];
    elseif state.physics_time < window_time * 8
        state.gyro = [angle2rate(state,1,0);0;0];
    elseif state.physics_time < window_time * 9
        % Pitch 5 deg/sec
        state.gyro = [0;deg2rad(5);0];
    elseif state.physics_time < window_time * 10
        state.gyro = [0;angle2rate(state,2,0);0];
    elseif state.physics_time < window_time * 11
        % Yaw 5 deg/sec
        state.gyro = [0;0;deg2rad(5)];
    elseif state.physics_time < window_time * 12
        if state.enable_print == 1
            state.enable_print = 0;
            fprintf('done!\n')
        end
        state.gyro = [0;0;angle2rate(state,3,0)];
    end
    
end

% Take a physics time step
function state = physics_step(pwm_in,state)
    % Depending on what the RC channel says, change attitude or rate
    % When RC 5 goes high, start the sequence. This is done from the GCS to
    % make sure that the autopilot is armed and ready to go before beginning.
    % Roll-Pitch-Yaw order

    % Settings for angle based inputs
    max_attitude_rate = 50;
    desired_attitude = 15;
    window_time = 5;
    
    if pwm_in(5) > 1500
        state.physics_time = state.physics_time + state.delta_t;
        % start sequence
        if state.physics_time < window_time * 6
            state = axis_angles(state,window_time,desired_attitude,max_attitude_rate);
        elseif state.physics_time < window_time * 12
            state = axis_rates(state,window_time);
        else
            state.gyro = [angle2rate(state,1,0);angle2rate(state,2,0);angle2rate(state,3,0)];
        end
    else
        % reset everything and go back to zero state
        state.physics_time = 0;
        state.enable_print = 1;
        state.gyro = [angle2rate(state,1,0);angle2rate(state,2,0);angle2rate(state,3,0)];
    end

    % Constrain to 2000 deg per second, this is what typical sensors max out at
    state.gyro = max(state.gyro,deg2rad(-2000));
    state.gyro = min(state.gyro,deg2rad(2000));

    % update the dcm, attitude, and gravity vector
    [state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);
    state.accel = state.dcm' * [0; 0; -state.gravity_mss];
end

function [dcm, euler] = rotate_dcm(dcm, ang)

    % rotate
    delta = [dcm(1,2) * ang(3) - dcm(1,3) * ang(2),         dcm(1,3) * ang(1) - dcm(1,1) * ang(3),      dcm(1,1) * ang(2) - dcm(1,2) * ang(1);
             dcm(2,2) * ang(3) - dcm(2,3) * ang(2),         dcm(2,3) * ang(1) - dcm(2,1) * ang(3),      dcm(2,1) * ang(2) - dcm(2,2) * ang(1);
             dcm(3,2) * ang(3) - dcm(3,3) * ang(2),         dcm(3,3) * ang(1) - dcm(3,1) * ang(3),      dcm(3,1) * ang(2) - dcm(3,2) * ang(1)];

    dcm = dcm + delta;

    % normalise
    a = dcm(1,:);
    b = dcm(2,:);
    error = a * b';
    t0 = a - (b *(0.5 * error));
    t1 = b - (a *(0.5 * error));
    t2 = cross(t0,t1);
    dcm(1,:) = t0 * (1/norm(t0));
    dcm(2,:) = t1 * (1/norm(t1));
    dcm(3,:) = t2 * (1/norm(t2));

    % calculate euler angles
    euler = [atan2(dcm(3,2),dcm(3,3)); -asin(dcm(3,1)); atan2(dcm(2,1),dcm(1,1))]; 

end

