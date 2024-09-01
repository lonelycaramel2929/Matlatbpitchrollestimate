clear all;clc;close all;
filename = 'pokuston.txt';
data = readmatrix(filename, 'Delimiter', '\t');
% Assign columns to variables
time_ms = data(:, 1); % Time in milliseconds
acc_x = data(:, 2); %ax [m/s^2]
acc_y = data(:, 3); %ax [m/s^2]
acc_z = data(:, 4); %ax [m/s^2]
gyro_x = data(:, 5); %gx [rad/s]
gyro_y = data(:, 6); %gy [rad/s]
gyro_z = data(:, 7); %gz [rad/s]

% Convert time to seconds
time_s = time_ms / 1000;

% Number of measurements
n = length(time_s);

% Kalman filter
initial_acc_x = mean(acc_x(10));
initial_acc_y = mean(acc_y(10));
initial_acc_z = mean(acc_z(10));

initial_roll = atan2(initial_acc_y, initial_acc_z);
initial_pitch = atan2(-initial_acc_x, sqrt(initial_acc_y^2 + initial_acc_z^2));
Q = 1e-5; % Process variance
R = 1e-2; % Measurement variance
P = [1; 1]; % Initial error estimate for pitch and roll
x_hat = [initial_pitch; initial_roll]; % Initial estimate of angles (pitch, roll)

% Initialize variables for results
pitch = zeros(n, 1);
roll = zeros(n, 1);

% Kalman filter
for i = 2:n
    % Calculate time interval
    dt = time_s(i) - time_s(i-1);

    % Measurements
    acc_angle_pitch = atan2(acc_y(i), acc_z(i)) * (180 / pi);
    acc_angle_roll = atan2(-acc_x(i), sqrt(acc_y(i)^2 + acc_z(i)^2)) * (180 / pi);

    % Prediction
    x_hat_minus = x_hat + [gyro_x(i) * dt; gyro_y(i) * dt];
    P_minus = P + Q;

    % Kalman gain
    K = P_minus ./ (P_minus + R);

    % Update
    x_hat = x_hat_minus + K .* ([acc_angle_pitch; acc_angle_roll] - x_hat_minus);
    P = (1 - K) .* P_minus;

    % Store results
    pitch(i) = x_hat(1);
    roll(i) = x_hat(2);
end

% Plot results
figure;
subplot(2, 1, 1);
plot(time_s, pitch, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch (°)');
title('Pitch Angle');
grid on;

subplot(2, 1, 2);
plot(time_s, roll, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Roll (°)');
title('Roll Angle');
grid on;
