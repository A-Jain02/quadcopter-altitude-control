% pid_attitude_control.m

load('quadcopter_sys.mat');

% Time and sim setup
dt = 0.01;
t = 0:dt:5;

% Desired targets
z_ref = 1.0;
roll_ref = 0;   % phi (ϕ)
pitch_ref = 0;  % theta (θ)
yaw_ref = 0;    % psi (ψ)

% PID gains
Kpz = 10;  Kiz = 5;  Kdz = 3;
Kproll = 0.6;  Kiroll = 0.05;  Kdroll = 0.3;
Kppitch = 0.6; Kipitch = 0.05; Kdpitch = 0.3;
Kpyaw = 1.2;   Kiyaw = 0.0;   Kdyaw = 0.5;

% Initialize
x = zeros(12,1);
x_hist = zeros(length(t), 12);
u_hist = zeros(length(t), 4);

% Integrators and previous errors
err_z_int = 0; err_roll_int = 0; err_pitch_int = 0; err_yaw_int = 0;
prev_z_err = 0; prev_roll_err = 0; prev_pitch_err = 0; prev_yaw_err = 0;

for i = 1:length(t)
    % Extract current states
    z = x(3); vz = x(6);
    roll = x(7); pitch = x(8); yaw = x(9);

    % --- Z axis control ---
    err_z = z_ref - z;
    err_z_int = err_z_int + err_z*dt;
    derr_z = (err_z - prev_z_err)/dt;
    u1 = m*(g + Kpz*err_z + Kiz*err_z_int + Kdz*derr_z);

    % --- Roll control ---
    err_roll = roll_ref - roll;
    err_roll_int = err_roll_int + err_roll*dt;
    derr_roll = (err_roll - prev_roll_err)/dt;
    u2 = Kproll*err_roll + Kiroll*err_roll_int + Kdroll*derr_roll;

    % --- Pitch control ---
    err_pitch = pitch_ref - pitch;
    err_pitch_int = err_pitch_int + err_pitch*dt;
    derr_pitch = (err_pitch - prev_pitch_err)/dt;
    u3 = Kppitch*err_pitch + Kipitch*err_pitch_int + Kdpitch*derr_pitch;

    % --- Yaw control ---
    err_yaw = yaw_ref - yaw;
    err_yaw_int = err_yaw_int + err_yaw*dt;
    derr_yaw = (err_yaw - prev_yaw_err)/dt;
    u4 = Kpyaw*err_yaw + Kiyaw*err_yaw_int + Kdyaw*derr_yaw;

    % Apply all 4 controls
    u = [u1; u2; u3; u4];

    % Update state
    dx = A*x + B*u;
    x = x + dx*dt;

    % Save data
    x_hist(i,:) = x';
    u_hist(i,:) = u';

    % Save prev errors
    prev_z_err = err_z;
    prev_roll_err = err_roll;
    prev_pitch_err = err_pitch;
    prev_yaw_err = err_yaw;
end

% Plot attitude
figure;
subplot(2,2,1);
plot(t, x_hist(:,3)); hold on; yline(z_ref, '--r');
ylabel('Z (m)'); title('Altitude');

subplot(2,2,2);
plot(t, x_hist(:,7)); hold on; yline(roll_ref, '--r');
ylabel('Roll (rad)'); title('Roll (ϕ)');

subplot(2,2,3);
plot(t, x_hist(:,8)); hold on; yline(pitch_ref, '--r');
ylabel('Pitch (rad)'); title('Pitch (θ)');

subplot(2,2,4);
plot(t, x_hist(:,9)); hold on; yline(yaw_ref, '--r');
ylabel('Yaw (rad)'); title('Yaw (ψ)');
saveas(gcf, 'attitude_control.png');         
