%With X = [x_base; y_base; z_base] and Z = [x_tip; y_tip; z_tip; horizangle_tip vertiangle_tip]\n');

clear; close all; clc;
global safe_limit;
global base_init;
global target;
global T;

%% Load Dataset
trial = 03;
extra = '';
folder = '2022-10-07';
name = 'trials_';
load(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'));

%% Configure simulationb
N = size(X,2);  % data size
safe_limit = 6;
base_init = base(:,1);

%% Loop key
k_key = find(key); % Samples when key was pressed
Nk = length(k_key);
T = zeros(1,Nk);
err = zeros(3,Nk);
for i=1:Nk
    k = k_key(i);   %sample
    T(:,i) = t(k);
    err(:,i) = tip(1:3,i) - target;
end

%% Loop all measurements
err_all = zeros(3,N);
for i=1:N
    err_all(:,i) = tip(1:3,i) - target; 
end


%% Plot control error to target
figure(1);
plot(t, err_all(1,:),'.-', t, err_all(3,:),'.-')
hold on;
plot_key('--g');
title('Error to target'),xlabel('time [s]'),ylabel('err [mm]'),  legend('X', 'Z')

% Control output
figure(2);
subplot(2,1,1)
plot(t, cmd(1,:), '.-')
hold on
plot_baseline_X('--k');
plot_safe_limit_X('k');
plot_key('--g');
title('Control output'),xlabel('time [s]'),ylabel('cmd X [mm]'),  legend('cmd_x', 'initial_x', 'safe limit')

subplot(2,1,2)
plot(t, cmd(3,:), '.-')
hold on
plot_baseline_Z('--k');
plot_safe_limit_Z('k');
plot_key('--g');
xlabel('time [s]'),ylabel('cmd Z [mm]'), legend('cmd_z', 'initial_z', 'safe limit')

% Tip and target
figure(3);
subplot(2,1,1)
plot(t, Z(1,:),'.-')
hold on
plot_target_X('--r')
plot_key('--g');
title('Tip vs Target'), xlabel('time [s]'),ylabel('X [mm]'), legend('tip','target')

subplot(2,1,2)
plot(t, Z(3,:), '.-')
hold on
plot_target_Z('--r')
plot_key('--g');

xlabel('time [s]'),ylabel('Z [mm]')

fprintf('Final error X [mm] = %0.4f\n', abs(err(1,end)));
fprintf('Final error Y [mm] = %0.4f\n', abs(err(2,end)));
fprintf('Final error Z [mm] = %0.4f\n', abs(err(3,end)));
fprintf('Final error 3D[mm] = %0.4f\n', sqrt(err(1,end)^2+err(2,end)^2+err(3,end)^2));
fprintf('Final error 2D[mm] = %0.4f\n', sqrt(err(1,end)^2+err(3,end)^2));



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
    global T
    for i=1:length(T)
        xline(T(i), line);
    end
end