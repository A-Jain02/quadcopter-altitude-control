% ekf_state_estimation.m

load('quadcopter_sys.mat');

% Sim time
dt = 0.01;
t = 0:dt:10;
n = length(t);

% Initialize true state
x_true = zeros(12,1);
x_true_hist = zeros(n, 12);

% Control inputs: hover
u = [m*g; 0; 0; 0];

% Simulate true system (no control yet)
for i = 1:n
    dx = A*x_true + B*u;
    x_true = x_true + dx*dt;
    x_true_hist(i,:) = x_true';
end

% Simulate sensor readings
gps_noise = 0.05 * randn(n,3);    % 5cm noise
imu_noise = 0.02 * randn(n,6);    % angular + linear acceleration

% Measured positions (GPS)
z_gps = x_true_hist(:,1:3) + gps_noise;

% Measured velocities and rates (IMU)
z_imu = x_true_hist(:,4:9) + imu_noise;

% Total measurements (z = [x y z vx vy vz roll pitch yaw])
z_meas = [z_gps z_imu];

% --- EKF Setup ---
% State estimate: 12x1
x_est = zeros(12,1);
x_est_hist = zeros(n,12);

% Covariance
P = eye(12)*0.1;

% Process noise
Q = eye(12)*0.01;

% Measurement matrix (identity)
H = eye(9,12);  % Only first 9 states observed
R = eye(9)*0.02;

for i = 1:n
    % --- Prediction ---
    dx = A*x_est + B*u;
    x_pred = x_est + dx*dt;
    P_pred = A*P*A' + Q;

    % --- Measurement update ---
    z = z_meas(i,:)';
    y = z - H*x_pred;                % innovation
    S = H*P_pred*H' + R;
    K = P_pred*H'/S;                 % Kalman gain

    x_est = x_pred + K*y;
    P = (eye(12) - K*H)*P_pred;

    x_est_hist(i,:) = x_est';
end

% --- Plot results ---
figure;
subplot(3,1,1);
plot(t, x_true_hist(:,1), 'b', t, x_est_hist(:,1), 'r--');
ylabel('X (m)');
legend('True','EKF Est');

subplot(3,1,2);
plot(t, x_true_hist(:,2), 'b', t, x_est_hist(:,2), 'r--');
ylabel('Y (m)');

subplot(3,1,3);
plot(t, x_true_hist(:,3), 'b', t, x_est_hist(:,3), 'r--');
ylabel('Z (m)');
xlabel('Time (s)');
sgtitle('EKF Estimation vs True Position');

saveas(gcf, 'EKF_estimation.png');  
