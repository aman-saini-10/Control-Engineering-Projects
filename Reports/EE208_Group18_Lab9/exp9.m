Kp = zeros(10, 3);
Ti = zeros(10, 3);
Td = zeros(10, 3);

%% DATA RETRIEVAL FROM SIMULINK MODEL 
% PROCESS REACTION CURVE METHOD FOR LAG TYPE SYSTEMS
for i=2:10
    fprintf('%s %i\n','Multiplicity',i);
    si=out.mul.signals.values(:,i);
    st=out.mul.time;    
    d=diff(si)./diff(st); %d is the slope of the signal
    [C,I]=max(d); %find maximum of the slope C and the index I where the slope is 
    y1=out.mul.signals.values(I,i);
    x1=out.mul.time(I);
    x=0:(x1+30);
    y=(C*(x-x1))+y1;
    %legend('signal','tangent')
    T=((si(end-1)-y1)/C)+x1;
    L=-(y1/C)+x1;
    T = T - L;
    Kp(i, (1:3)) = [(T)/L;0.9*T/L;1.2*T/L]; % proportional gain for P, Pi, Pid
    Ti(i, (1:3)) = [-1;L/0.3;2*L]; % time constant for I 
    Td(i, (1:3)) = [0;0;0.5*L]; % time constant for D
    %hold on
    %plot(st,si)
    %plot(x,y);
    %hold on
end

%% Dependencies on Multiplicity
mul = (1:10); % array for multiplicity
scatter(mul,Kp(:, 1)); grid on; grid minor; hold on; % P
scatter(mul,Kp(:, 2)); % PI
scatter(mul,Kp(:, 3)); % PID 
legend('P','PI','PID'); hold off;
figure;
scatter(mul,Ti(:, 2));hold on; % PI;
scatter(mul,Ti(:, 3)); %PID;
legend('PI', 'PID'); grid on; grid minor;
hold off;
figure; 
scatter(mul, Td(:, 3)); grid on; grid minor;
legend('Td vs mul for PID')

%% P CONTROL
s = tf('s'); H = 1/(s+1.5);
settling_Time = zeros(1, 10);
Osettling_Time = zeros(1,10);
oshoot = zeros(1, 10);
rise_time = zeros(1, 10);
Orise_time = zeros(1, 10);
figure;
for i = 2:10
    controller = Kp(i, 1);
    cltf = ((controller)*(H^i))/(1 + controller*(H^i));
%     step(cltf); hold on;
    A = stepinfo(cltf);
    B = stepinfo(H^i);
    settling_Time(i) = A.SettlingTime;
    Osettling_Time(i) = B.SettlingTime;
    rise_time(i) = A.RiseTime;
    Orise_time(i) = B.RiseTime;
    oshoot(i) = A.Overshoot;
end 
% grid on; grid minor; 
% legend('k = 2','k = 3','k = 4','k = 5','k = 6','k = 7','k = 8','k = 9','k = 10');
% hold off;
figure;
scatter(mul, settling_Time); hold on; grid on; grid minor; 
scatter(mul, Osettling_Time); legend('P controlled closed loop system', 'Open loop system');
hold off;
figure;
scatter(mul, rise_time); hold on; grid on; grid minor; 
scatter(mul, Orise_time); legend('P controlled closed loop system', 'Open loop system');
hold off;
figure; 
scatter(mul, oshoot); grid on; grid minor;

%% PI control
figure;
for i = 2:10
    controller = Kp(i, 2) + Kp(i, 2)/(s*Ti(i, 2)); 
    cltf = ((controller)*(H^i))/(1 + controller*(H^i));
    step(cltf); hold on;
    A = stepinfo(cltf);
    B = stepinfo(H^i);
    settling_Time(i) = A.SettlingTime;
    Osettling_Time(i) = B.SettlingTime;
    rise_time(i) = A.RiseTime;
    Orise_time(i) = B.RiseTime;
    oshoot(i) = A.Overshoot;
end 
grid on; grid minor;
legend('k = 2','k = 3','k = 4','k = 5','k = 6','k = 7','k = 8','k = 9','k = 10');
hold off;
figure;
scatter(mul, settling_Time); hold on; grid on; grid minor; 
scatter(mul, Osettling_Time); legend('PI controlled closed loop system', 'Open loop system');
z = gca();
set(z, 'yscale', 'log');
hold off;
figure;
scatter(mul, rise_time); hold on; grid on; grid minor; 
scatter(mul, Orise_time); legend('PI controlled closed loop system', 'Open loop system');
z = gca();
set(z, 'yscale', 'log');
hold off;
figure; 
scatter(mul, oshoot); grid on; grid minor;

%% PID CONTROL 
figure;
for i = 2:10
    controller = Kp(i, 3)*(1 + 1/(s*Ti(i, 3)) + s*Td(i, 3)); 
    cltf = ((controller)*(H^i))/(1 + controller*(H^i));
    step(cltf); hold on;
    A = stepinfo(cltf);
    B = stepinfo(H^i);
    settling_Time(i) = A.SettlingTime;
    Osettling_Time(i) = B.SettlingTime;
    rise_time(i) = A.RiseTime;
    Orise_time(i) = B.RiseTime;
    oshoot(i) = A.Overshoot;
end 
grid on; grid minor;
legend('k = 2','k = 3','k = 4','k = 5','k = 6','k = 7','k = 8','k = 9','k = 10');
hold off;
figure;
scatter(mul, settling_Time); hold on; grid on; grid minor; 
scatter(mul, Osettling_Time); legend('PID controlled closed loop system', 'Open loop system');
z = gca();
set(z, 'yscale', 'log');
hold off;
figure;
scatter(mul, rise_time); hold on; grid on; grid minor; 
scatter(mul, Orise_time); legend('PID controlled closed loop system', 'Open loop system');
z = gca();
set(z, 'yscale', 'log');
hold off;
figure; 
scatter(mul, oshoot); grid on; grid minor;
