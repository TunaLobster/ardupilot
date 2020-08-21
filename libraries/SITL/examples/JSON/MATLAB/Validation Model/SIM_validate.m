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

% Take a physics time step
function state = physics_step(pwm_in,state)
    % Depending on what the RC channel says, change attitude or rate
    % When RC 5 goes high, start the sequence. This is done from the GCS to
    % make sure that the autopilot is armed and ready to go before beginning.
    
    % Need to change this.
    % - Rotate at 200 deg/sec until attitude is greater than desired for
    % attitude.
    % - Rotate at 5 deg/s and find attidue that matches that step.
    
    if pwm_in(5) > 1500
        state.physics_time = state.physics_time + state.delta_t;
        % start sequence
        % attidue angles first. Start from zero each time.
        % Roll-Pitch-Yaw order
        window_time = 10;
        if state.physics_time < window_time
            if state.enable_print == 1
                state.enable_print = 0;
                fprintf('start motion\n')
            end
            % Roll 15 deg
            state.attitude = [deg2rad(15);0;0];
        elseif state.physics_time < window_time * 2
            state.enable_print = 1;
            state.attitude = [0;0;0];
        elseif state.physics_time < window_time * 3
            % Pitch 15 deg
            state.attitude = [0;deg2rad(15);0];
        elseif state.physics_time < window_time * 4
            state.attitude = [0;0;0];
        elseif state.physics_time < window_time * 5
            % Yaw 15 deg
            state.attitude = [0;0;deg2rad(15)];
        elseif state.physics_time < window_time * 6
            state.attitude = [0;0;0];
        % state.physics_time rates
        elseif state.physics_time < window_time * 7
            % Roll 5 deg/sec
            state.gyro = [deg2rad(5);0;0];
        elseif state.physics_time < window_time * 8
            state.gyro = [0;0;0];
        elseif state.physics_time < window_time * 9
            % Pitch 5 deg/sec
            state.gyro = [0;deg2rad(5);0];
        elseif state.physics_time < window_time * 10
            state.gyro = [0;0;0];
        elseif state.physics_time < window_time * 11
            % Yaw 5 deg/sec
            state.gyro = [0;0;deg2rad(5)];
        elseif state.physics_time < window_time * 12
            if state.enable_print == 1
                state.enable_print = 0;
                fprintf('done!\n')
            end
            state.gyro = [0;0;0];
        end
        % update the dcm and attitude
        %[state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);
    else
        % reset everything and go back to init state
        state.physics_time = 0;
        state = init(state);
    end
    % set the gravity vector
    state.accel = [state.gravity_mss * sin(state.attitude(2))
                   state.gravity_mss * cos(state.attitude(2)) * sin(state.attitude(1))
                   state.gravity_mss * cos(state.attitude(2)) * cos(state.attitude(1))];
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

