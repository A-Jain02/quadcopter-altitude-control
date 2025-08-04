% simulate_open_loop.m

load('quadcopter_sys.mat');

% Initial state
x0 = zeros(12,1);

% Step input (constant upward thrust only)
u = [9.81; 0; 0; 0];  % Balance gravity

t = 0:0.01:5;  % 5 second sim
[y, t_out, x_out] = lsim(quad_sys, repmat(u', length(t), 1), t, x0);

plot(t_out, x_out(:,3)); % plot Z position
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Open-loop Altitude');
