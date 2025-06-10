%% 初始化飞行器状态
StopTime = 500;
dt = 0.02;
% pursuer
psi1 = 0;
v1 = 60;
x1 = 0;
y1 = 0;

% target
psi1_t = 2*pi * (rand - 0.5);
a1_t = 0;
v1_t = 60;
radius = 1e3 * (3 + rand);
angle = 2*pi * rand;
x1_t = radius * cos(angle);
y1_t = radius * sin(angle);
phi1_t = 0;