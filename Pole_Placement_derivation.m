M = 0.250; % Cart mass (kg)
m = 0.05; % Pendulum mass (kg)
g = 9.81; % Gravitational acceleration (m/s^2)
l = 0.42; % Pendulum length (m)
b = 0.1; % Damping/friction coefficient (Ns/m)

% State-space matrices with damping
A = [0, 1, 0, 0;
     0, -b/M, -m*g/M, 0;
     0, 0, 0, 1;
     0, b/(M*l), (M+m)*g/(M*l), 0];

B = [0; 1/M; 0; -1/(M*l)];

% Open-loop poles
disp('Open-loop poles:');
disp(eig(A));

% Desired closed-loop poles
desired_poles = [-2 -3 -4 -5];

% Compute feedback gain
K = place(A, B, desired_poles);

% Closed-loop system
A_cl = A - B * K;

% Simulate step response
sys_cl = ss(A_cl, B, eye(4), zeros(4, 1));
step(sys_cl);
title('Closed-Loop Step Response');
