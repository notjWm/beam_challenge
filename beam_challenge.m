clear
clc

%% Parameters
beam_l = 10; % m
dt = 0.01; % s
Kp = 10;
Kd = 7;
Ki = 3;
total_t = 30; % s
g = 9.81; % m/s^2 

time_vec = zeros(total_t/dt, 1);
x_vec = zeros(total_t/dt, 1);
v_vec = zeros(total_t/dt, 1);
theta_vec = zeros(total_t/dt, 1);
theta_vec(1) = 0; % smoothes out the curves
% theta_vec(1) = deg2rad(rand * 45); % if you wanted to start with a random
% angle from 0 degrees to 45 degrees
error = zeros(total_t/dt, 1);

% Initial conditions
initial_x = rand * beam_l; % m (initial position)
x_vec(1) = initial_x;
desired_x = rand * beam_l; % m (target position)
error(1) = desired_x - initial_x;

% Checks to see if desired location has been updated
last_time_update = 0;

%% Control Loop
for i = 2:numel(time_vec)

    % PID control
    error(i) = desired_x - x_vec(i-1);
    [theta, integral_error] = pid_control(error(i), dt, Kp, Ki, Kd); 

    % Update theta (control signal)
    theta_vec(i) = theta;

    % Calculate acceleration
    acceleration = g * sin(theta);

    % Update velocity and position
    v_vec(i) = v_vec(i-1) + acceleration * dt; 
    x_vec(i) = x_vec(i-1) + v_vec(i) * dt; 

    time_vec(i) = i * dt; 
end

%% Plotting
figure;
subplot(2,2,1);
plot(time_vec, x_vec, 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, desired_x, 'r--', 'LineWidth', 1.5); % Plot the desired_x as a red dashed line
title("Ball Position vs Time");
xlabel("Time (s)");
ylabel("Position (meters)");
legend('X Position (meters)', 'Desired Position');

subplot(2,2,2);
plot(time_vec, v_vec, 'b', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.5); % Plot velocity = 0 as a red dashed line
title("Ball Velocity vs Time");
xlabel("Time (s)");
ylabel("Velocity (m/s)");
legend('Velocity', 'Velocity = 0');

subplot(2,2,4);
plot(time_vec, error, 'b', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.5); % Plot error = 0 as a red dashed line
title("Error vs Time");
xlabel("Time (s)");
ylabel("Error (meters)");
legend('Error', 'Error = 0');

%% PID Control Function
function [theta, integral_error] = pid_control(error, dt, Kp, Ki, Kd)
    persistent integral_error_p;  
    persistent previous_error; 
    
    if isempty(integral_error_p)
        integral_error_p = 0;
        previous_error = 0;
    end
    
    % Compute PID components
    derivative_error = (error - previous_error) / dt;
    integral_error_p = integral_error_p + error * dt;
    
    % Mathematical Model
    theta = Kp * error + Ki * integral_error_p + Kd * derivative_error;
    
    previous_error = error;

    integral_error = integral_error_p;
end
