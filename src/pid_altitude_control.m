% pid_altitude_control.m

load('quadcopter_sys.mat');

% Indices: 
% x(3) = z
% x(6) = vz

% Desired altitude setpoint
z_ref = 1.0; % meters

% PID gains for altitude
Kp = 10;
Ki = 5;
Kd = 3;

dt = 0.01;
t = 0:dt:5;

% Initial state
x = zeros(12,1);
x_hist = zeros(length(t), 12);
u_hist = zeros(length(t), 4);
z_err_int = 0;
prev_err = 0;

for i = 1:length(t)
    z = x(3);      % current altitude
    vz = x(6);     % vertical velocity
    err = z_ref - z;
    
    z_err_int = z_err_int + err*dt;
    derr = (err - prev_err)/dt;
    
    % PID for thrust (U1)
    u1 = m*(g + Kp*err + Ki*z_err_int + Kd*derr);
    u = [u1; 0; 0; 0];  % Only thrust applied
    
    % State update using linear system
    dx = A*x + B*u;
    x = x + dx*dt;
    
    % Save
    x_hist(i,:) = x';
    u_hist(i,:) = u';
    
    prev_err = err;
end

% Plot altitude
figure;
plot(t, x_hist(:,3), 'b', 'LineWidth', 1.5);
hold on;
yline(z_ref, '--r');
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Control using PID');
legend('Altitude', 'Target');

saveas(gcf, 'altitude_control.png');          
