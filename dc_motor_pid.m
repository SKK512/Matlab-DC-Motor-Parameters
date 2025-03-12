% Define DC Motor Parameters
J = 0.01;   % Inertia
B = 0.1;    % Damping
K = 0.01;   % Motor Constant
R = 1;      % Resistance
L = 0.5;    % Inductance

% Create Transfer Function
s = tf('s');
G = K / ( (J*s + B) * (L*s + R) + K^2 );

% Plot Open-Loop Response
step(G);
title('Open-Loop Response of DC Motor');
grid on;
% Define PID Controller
Kp = 50;    % Proportional Gain
Ki = 100;   % Integral Gain
Kd = 5;     % Derivative Gain

C = pid(Kp, Ki, Kd);  % Create PID Controller

% Apply PID to the Motor System
T = feedback(C*G, 1);  % Closed-Loop System

% Plot the Controlled Response
figure;
step(T);
title('DC Motor Response with PID Control');
grid on;