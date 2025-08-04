% pid_position_control.m

load('quadcopter_sys.mat');

dt = 0.01;
t = 0:dt:10;

% Position targets
x_ref = 1.0;
y_ref = 1.0;
z_ref = 1.0;
yaw_ref = 0;

% PID gains for outer loop (x, y → pitch, roll)
Kpx = 0.6; Kdx = 0.3;
Kpy = 0.6; Kdy = 0.3;

% PID gains for inner loop
Kpz = 10; Kiz = 5; Kdz = 3;
Kproll = 0.8; Kiroll = 0.05; Kdroll = 0.4;
Kppitch = 0.8; Kipitch = 0.05; Kdpitch = 0.4;
Kpyaw = 1.2; Kdyaw = 0.6;

% Initialize
x = zeros(12,1);
x_hist = zeros(length(t), 12);
u_hist = zeros(length(t), 4);

% Integrators
err_z_int = 0; err_roll_int = 0; err_pitch_int = 0; err_yaw_int = 0;
prev_z_err = 0; prev_roll_err = 0; prev_pitch_err = 0; prev_yaw_err = 0;

for i = 1:length(t)
    % State extraction
    xpos = x(1); vx = x(4);
    ypos = x(2); vy = x(5);
    z = x(3); vz = x(6);
    roll = x(7); pitch = x(8); yaw = x(9);

    % --- Outer loop: x/y position → pitch/roll references ---
    err_x = x_ref - xpos;
    derr_x = -vx;
    pitch_ref = -(Kpx * err_x + Kdx * derr_x);  % Forward = negative pitch

    err_y = y_ref - ypos;
    derr_y = -vy;
    roll_ref = (Kpy * err_y + Kdy * derr_y);    % Right = positive roll

    % --- Altitude PID (same as before) ---
    err_z = z_ref - z;
    err_z_int = err_z_int + err_z*dt;
    derr_z = (err_z - prev_z_err)/dt;
    u1 = m*(g + Kpz*err_z + Kiz*err_z_int + Kdz*derr_z);

    % --- Roll PID (inner loop) ---
    err_roll = roll_ref - roll;
    err_roll_int = err_roll_int + err_roll*dt;
    derr_roll = (err_roll - prev_roll_err)/dt;
    u2 = Kproll*err_roll + Kiroll*err_roll_int + Kdroll*derr_roll;

    % --- Pitch PID ---
    err_pitch = pitch_ref - pitch;
    err_pitch_int = err_pitch_int + err_pitch*dt;
    derr_pitch = (err_pitch - prev_pitch_err)/dt;
    u3 = Kppitch*err_pitch + Kipitch*err_pitch_int + Kdpitch*derr_pitch;

    % --- Yaw PID ---
    err_yaw = yaw_ref - yaw;
    derr_yaw = (err_yaw - prev_yaw_err)/dt;
    u4 = Kpyaw*err_yaw + Kdyaw*derr_yaw;

    % Control vector
    u = [u1; u2; u3; u4];

    % State update
    dx = A*x + B*u;
    x = x + dx*dt;

    % Save
    x_hist(i,:) = x';
    u_hist(i,:) = u';

    % Update prev errors
    prev_z_err = err_z;
    prev_roll_err = err_roll;
    prev_pitch_err = err_pitch;
    prev_yaw_err = err_yaw;
end

% Plot x-y position
figure;
plot(x_hist(:,1), x_hist(:,2), 'b', 'LineWidth', 1.5);
hold on;
plot(x_ref, y_ref, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Quadcopter Horizontal Trajectory');
legend('Trajectory', 'Target');
axis equal;
grid on;

saveas(gcf, 'position_control.png');  
