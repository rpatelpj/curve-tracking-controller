%% Path Following Control Law Demonstration
% Raj Patel
close all; restoredefaultpath; clc; clear;
addpath(genpath('~/Documents/MATLAB/Add-Ons/Toolboxes/Robotics Toolbox for MATLAB'));

%% Choose Parameters

% Environment
field_scale = 7;
field_margin = 1;
field_size = field_scale.*[-1, 1, -1, 1];

% Robot
v_o = 1.2;
acc = 1;
turn_rt_lim = 2.3;
frame_shift = 0.2;

% Controller
k_rho = 0.7;
k_phi = sqrt(4.*k_rho) + 0.2;
rho_o = 0.2;

% Path (use 'path_preprocessing.m')
f = @(x) 0.01.*x.^5 - 0.27.*x.^3 + 0.14.*x.^2 + 1.2.*x - 3;

% Video
anim_rate = 10;

% Initial Condition
% x0 = [-6, -4, pi./3]; % Start near bottom left corner
% x0 = [-4, 2, 0]; % Start near extrema of path
% x0 = [-4, 2, pi./4]; % Start near extrema of path 2
% x0 = [0, 4, pi]; % Start top center
x0 = [0, 2, -pi/4]; % Start center
% x0 = [-4, -5, pi./3]; % Start on right side of path

%% Simulate

ctrl_sim = sim('path_following_controller', 20);

%% Animation

fig = figure;
axis(field_size);
hold on;
xyzlabel;
grid on;

x = field_size(1):0.01:field_size(2);
y = f(x);
plot(x, y, 'k--');

q_R = ctrl_sim.yout{1}.Values.Data;
plot(q_R(1:anim_rate:end, 1), q_R(1:anim_rate:end, 2), 'b.');
plot_vehicle(q_R(1:anim_rate:end, :), 'movie', 'videos/sim.mp4');