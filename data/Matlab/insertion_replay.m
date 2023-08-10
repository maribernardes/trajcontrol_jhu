%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');

clear; close all; clc;
global safe_limit;
global base_init;
global target;
global T_key;

%% Load Dataset
trial = 0;
extra = '';
folder = 'jhu_demo';
name = 'my_data_';

load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'));

%% Configure simulationb
N = size(X,2);  % data size

%% Loop key
Nk = length(key_index); % Number of key hits
T_key = zeros(1,Nk);        % Time of a key hit
for i=1:Nk
    k = key_index(i);   
    T_key(:,i) = t(k);
end


%% Plot robot frame
figure(3);
subplot(3,1,1)
plot(t, tip_r(1,:),'r.-')
hold on
plot(t, base_r(1,:),'b.-')
plot_key('--g');
title('Positions [robot frame]'), xlabel('time [s]'),ylabel('X [mm]'), legend('tip', 'base')

subplot(3,1,2)
plot(t, tip_r(2,:), 'r.-')
hold on
plot(t, base_r(2,:),'b.-')
plot_key('--g');
xlabel('time [s]'),ylabel('Y [mm]')

subplot(3,1,3)
plot(t, tip_r(3,:), 'r.-')
hold on
plot(t, base_r(3,:),'b.-')
plot_key('--g');
xlabel('time [s]'),ylabel('Z [mm]')


%% Plot needle frame
figure(4);
subplot(3,1,1)
plot(t, tip_n(1,:),'.-')
hold on
plot(t, base_n(1,:),'.-')
plot_key('--g');
title('Positions [needle frame]'), xlabel('time [s]'),ylabel('X [mm]'), legend('tip', 'base')

subplot(3,1,2)
plot(t, tip_n(2,:), 'r.-')
hold on
plot(t, base_n(2,:),'b.-')
plot_key('--g');
xlabel('time [s]'),ylabel('Y [mm]')

subplot(3,1,3)
plot(t, tip_n(3,:), 'r.-')
hold on
plot(t, base_n(3,:),'b.-')
plot_key('--g');
xlabel('time [s]'),ylabel('Z [mm]')

function plot_target(line)
    global target;
    plot_target_X(line)
    plot_target_Z(line)
end

function plot_target_X(line)
    global target;
    yline(target(1), line)
end

function plot_target_Z(line)
    global target;
    yline(target(3), line)
end

function plot_baseline(line)
    global base_init;
    plot_baseline_X(line)
    plot_baseline_Z(line)
end

function plot_baseline_X(line)
    global base_init;
    yline(base_init(1), line)
end

function plot_baseline_Z(line)
    global base_init;
    yline(base_init(3), line)
end

function plot_safe_limit(line)
    global base_init;
    global safe_limit;
    plot_safe_limit_X(line)
    plot_safe_limit_Z(line)
end

function plot_safe_limit_X(line)
    global base_init;
    global safe_limit;
    yline(base_init(1)+safe_limit,line)
    yline(base_init(1)-safe_limit,line)
end

function plot_safe_limit_Z(line)
    global base_init;
    global safe_limit;
    yline(base_init(3)+safe_limit,line) 
    yline(base_init(3)-safe_limit,line)
end

function plot_key(line)
    global T_key
    for i=1:length(T_key)
        xline(T_key(i), line);
    end
end