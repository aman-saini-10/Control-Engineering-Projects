A = [-0.14 -0.69 0; -0.19 -0.048 0; 0 -1 0];
B = [0.056; -0.23; 0];
C = [1 0 0; 0 1 0; 0 0 1];
D = [0; 0; 0];
% outputs => sway speed, yaw angle
% state space => sway speed, yaw angle, yaw rate
states = {'sway speed', 'yaw angle', 'yaw rate'};
output = {'sway speed', 'yaw angle', 'yaw rate'};
input = {'rudder angle'};
sys = ss(A, B, C, D, 'statename', states,...
    'inputname', input,...
    'outputname', output); % state space model
t = 0:0.01:100;
i = sqrt(-1);
P = [-0.5 -0.32 0];
K = place(A, B, P);
syscl = ss(A-B*K, B, C, D, 'statename', states,...
    'inputname', input,...
    'outputname', output);
G = tf(sys);
rank(ctrb(syscl))
rank(obsv(syscl))
impulse(sys, t), grid; %original open loop system impulse response
figure; step(syscl, t), grid;
figure; impulse(syscl, t), grid; 

Q = [1 0 0;
     0 1 0;
     0 0 1];
R = 0.01;
K2 = lqr(A, B, Q, R);
syscl2 = ss(A- B*K2, B, C, D, 'statename', states,...
    'inputname', input,...
    'outputname', output);
G2 = tf(syscl2);
rank(ctrb(syscl2))
rank(obsv(syscl2))
figure; step(syscl2, t), grid;
figure; impulse(syscl2, t), grid; 
