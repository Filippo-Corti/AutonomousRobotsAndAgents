%% EKF for 2D Unicycle Robot Tracking with Time-Varying Controls
clear; clc; close all; rng(42);

%% 1. Initialization and Setup

% Uncertainty Models (Variances)
Q = diag([0.03, 0.03, 0.05].^2);  % Q = Uncertainty in Motion (X, Y, Theta)
R = diag([0.1, 0.1].^2);        % R = Uncertainty in GPS Sensor (X, Y)

% General Settings
dt = 0.1;                       % Time step (seconds)
t_end = 10;                     % Total simulation time
time = 0:dt:t_end;              % Time instants (1 to 10)
N = length(time);               % Number of time steps (100)

% Control Input u: for each t [linear velocity (v); angular velocity (omega)]
u = zeros(2, N);
for i = 1:N
    % We program the robot to drive a specific path
    if time(i) < 3.0
        u(1,i) = 1.0; u(2,i) = 0.0; % Drive straight for steps 1-29
    elseif time(i) < 7.0
        u(1,i) = 0.8; u(2,i) = 1.0; % Curve left for steps 30-69
    else
        u(1,i) = 1.2; u(2,i) = -0.5; % Speed up and curve right for steps 70-101
    end
end

% Memory allocations for robot's future locations
x_true = zeros(3, N); % The actual physical position (hidden from the robot)
x_dr = zeros(3, N);   % Dead Reckoning: Blindly guessing based on wheel speed
x_ekf = zeros(3, N);  % The EKF estimate (The smart fusion of math + sensors)

% P_ekf is our "Confidence Matrix" (Covariance).
% Initially, we are slightly unsure of our starting position.
P_ekf = diag([0.1, 0.1, 0.1]);

% Memory for our noisy GPS readings [X; Y]
Z_history = zeros(2, N);

%% 2. Simulation Loop

for k = 2:N

    % The command we sent to the motors at the previous time step
    u_k = u(:, k-1);

    % =======================================================

    % 1) SIMULATE THE REAL WORLD
    % We add physical noise to the motion to simulate wheel slip.
    process_noise = mvnrnd([0 0 0], Q)'; % Noise according to our motion uncertainty Q

    % Run standard unicycle kinematics (non-linear)
    % Next X = Current X + (velocity * cos(angle) * time) + slip
    x_true(1,k) = x_true(1,k-1) + u_k(1)*cos(x_true(3,k-1))*dt + process_noise(1);  % x
    x_true(2,k) = x_true(2,k-1) + u_k(1)*sin(x_true(3,k-1))*dt + process_noise(2);  % y
    x_true(3,k) = x_true(3,k-1) + u_k(2)*dt + process_noise(3);                     % theta

    % Simulate the GPS reading: Actual position + sensor inaccuracy
    meas_noise = mvnrnd([0 0], R)'; % Noise according to our sensor uncertainty R
    z = [x_true(1,k); x_true(2,k)] + meas_noise;
    Z_history(:,k) = z;

    % =======================================================

    % 2) USE DEAD RECKONING (The naive approach)
    % Calculating position using ONLY the math model, assuming no slip (unicycle kinematics).
    % Notice this has no noise added, so it will inevitably drift from reality.
    x_dr(1,k) = x_dr(1,k-1) + u_k(1)*cos(x_dr(3,k-1))*dt;                           % x
    x_dr(2,k) = x_dr(2,k-1) + u_k(1)*sin(x_dr(3,k-1))*dt;                           % y
    x_dr(3,k) = x_dr(3,k-1) + u_k(2)*dt;                                            % theta

    % =======================================================

    % 3) USE EXTENDED KALMAN FILTER (The smart approach)
    x_prev = x_ekf(:, k-1);

    % STEP 1: PREDICTION ("Where do I think I am based on my wheels?") -> Once again, unicycle kinematics
    x_pred = zeros(3,1);
    x_pred(1) = x_prev(1) + u_k(1)*cos(x_prev(3))*dt;  
    x_pred(2) = x_prev(2) + u_k(1)*sin(x_prev(3))*dt;
    x_pred(3) = x_prev(3) + u_k(2)*dt;

    % Linearizing the model: The Jacobian 'F' takes the derivative of the
    % non-linear motion equations with respect to the state.
    % Intuition: "If my heading (Theta) was slightly wrong a moment ago,
    % how much does that affect my X and Y position now?"
    F = [1, 0, -u_k(1)*sin(x_prev(3))*dt;
        0, 1,  u_k(1)*cos(x_prev(3))*dt;
        0, 0,  1];

    % Predict our new uncertainty. It grows because moving adds process noise (Q).
    P_pred = F * P_ekf * F' + Q;

    % STEP 2: UPDATE ("What does the sensor say, and who do I trust?")
    % Map our predicted state (X,Y,Theta) to what the sensor measures (X,Y only).
    z_pred = [x_pred(1); x_pred(2)];

    % 'H' is the observation matrix. It just extracts X and Y.
    H = [1, 0, 0;
        0, 1, 0];

    % 'y' is the Innovation: The difference between the actual GPS reading
    % and where we expected the GPS reading to be.
    y = z - z_pred;

    % 'S' is the Innovation Covariance: Total uncertainty (Predicted + Sensor)
    S = H * P_pred * H' + R;

    % 'K' is the Kalman Gain: The magic ratio.
    % If sensor noise (R) is high, K is small (trust the prediction).
    % If prediction uncertainty (P) is high, K is large (trust the sensor).
    K = P_pred * H' / S;

    % Final State Estimate: Our prediction, corrected by the scaled innovation.
    x_ekf(:, k) = x_pred + K * y;

    % Final Uncertainty Estimate: Uncertainty shrinks because we gained information.
    P_ekf = (eye(3) - K * H) * P_pred;
end

%% 3. Visualization
% (Visualization code remains unchanged, but note the interactive legend)
figure('Name', 'EKF - Diff. drive robot', 'Color', 'w');
hold on; grid on; axis equal;

h_true = plot(x_true(1,:), x_true(2,:), '-', 'Color', [0 0.5 0], 'LineWidth', 2, 'DisplayName', 'Ground truth');
h_dr   = plot(x_dr(1,:), x_dr(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Dead reckoning');
h_gps  = plot(Z_history(1,2:end), Z_history(2,2:end), 'r.', 'MarkerSize', 8, 'DisplayName', 'Noisy GPS');
h_ekf  = plot(x_ekf(1,:), x_ekf(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'EKF');

plot(x_true(1,1), x_true(2,1), 'ko', 'MarkerSize', 10, 'LineWidth', 1.5, 'MarkerFaceColor', 'y', 'DisplayName', 'Start location');
xlabel('x'); ylabel('y'); title('EKF - Diff. drive robot');

axis manual;

% Start with only the Ground Truth visible. Click the legend to show the others.
h_dr.Visible  = 'off';
h_gps.Visible = 'off';
h_ekf.Visible = 'off';

lgd = legend('Location', 'best');
lgd.ItemHitFcn = @(~, event) toggleLine(event);

function toggleLine(event)
if strcmp(event.Peer.Visible, 'on')
    event.Peer.Visible = 'off';
else
    event.Peer.Visible = 'on';
end
end