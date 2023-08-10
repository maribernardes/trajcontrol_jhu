clear; close all; clc;

%% Run this to create matlab .mat files from csv files saved by ROS2 save_file node

%% Get .csv file into table T
trial = 0;
extra = '';
folder = 'jhu_demo';
name = 'my_data_';

%% Load Dataset
% Read data from table
T = readtable(strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'.csv'), 'HeaderLines',1);

% Define sample sizes
i = find(T{:,3}, 1,'first');   % First sample (when "target" is published) => First SPACE key is hit
if isempty(i)
    i=1;
end
j = height(T);                 % Last sample

%% Extract values from table
% Timestamp
sec = T{i:j,1};
nanosec = T{i:j,2};

% Target and skin_entry [robot]
target_r = T{i:j,3:5};
skin_entry_r = T{i:j,8:10};

% Tip [needle] and [robot]
posTip_n = T{i:j,13:15};
qTip_n = T{i:j,16:19};
posTip_r = T{i:j,22:24};
qTip_r = T{i:j,25:28};

% Base [needle] and [robot]
posBase_n = T{i:j,31:33};       % [StageX -DepthY StageZ]
qBase_n = T{i:j,34:37};         % Identity

posBase_r = T{i:j,40:42};       % Base in needle frame (defined at initial experiment pose)
qBase_r = T{i:j,43:46};         % Relative rotation between needle and robot frame

% Jacobian (by lines)
J1 = T{i:j,49:51};
J2 = T{i:j,52:54};
J3 = T{i:j,55:57};
J4 = T{i:j,58:60};
J5 = T{i:j,61:63};

depth = T{i:j,75};
key = T{i:j,78};

%% Save in matlab
% Control outputs
cmd = T{i:j,66:68}';
% Target
target = T{i,3:5}';
% Base initial position
base_init = T{i,40:42}';
% Stage
stage = 1000*T{i:j,71:72}';

%% Define variables to be saved in .mat
% Total number of samples
k = length(sec);
%Time vector
t = zeros(1,k);
% Aurora sensor values (in vector format [x, y, z, qw, qx, qy, qz])
tip_n = zeros(7,k);
base_n = zeros(3,k);
% Sensor values in stage frame - published by ros2 topics (in vector format)
tip_r = zeros(7,k);
base_r = zeros(3,k);
% Sensor values in stage frame - obtained with transform in matlab (in vector format)
X = zeros(3,k);
Z = zeros(5,k);
% Jacobian matrix
J = cell(1,k);


%% Separate when controller was ON
[~, initcmd] = find(cmd, 1,'first');
for i=1:initcmd-1
    cmd(:,i) = base_init; 
end

for i=1:k
    %Set time vector in seconds
    t(i) = sec(i)-sec(1) + (nanosec(i)-nanosec(1))*1e-9;
    
    tip_n(:,i) = [posTip_n(i,:) qTip_n(i,:)]';
    tip_r(:,i) = [posTip_r(i,:) qTip_r(i,:)]';
    base_n(:,i) = posBase_n(i,1:3)';
    base_r(:,i) = posBase_r(i,1:3)';
    
    [horiz, vert]= get_needle_angles(qTip_r(i,:));
    Z(:,i) = [posTip_r(i,:), horiz, vert];
    X(:,i) = posBase_r(i,1:3)';
    
    J{i} = [J1(i,:); J2(i,:); J3(i,:); J4(i,:); J5(i,:)];
end

% Indexes with key being hit
key_index = find(key);

% Load predictions from mat file
matfile_name = strcat(folder,'/',name,num2str(trial,'%2.2d'),extra,'_pred.mat');
if isfile(matfile_name)
    load(matfile_name);

    %% Configure simulationb
    N = size(u_pred,1);  % data size

    %% Loop key
    for i=1:N
        ux = u_pred(i,:,1);
        uz = u_pred(i,:,2);
        up{i} = [ux' uz'];

        yx = y_pred(i,:,1);
        yy = y_pred(i,:,2);
        yz = y_pred(i,:,3);  

        if size(y_pred,3) == 3
            yp{i} = [yx' yy' yz']; 
        else
            yv = y_pred(i,:,4);  
            yh = y_pred(i,:,5);  
            yp{i} = [yx' yy' yz' yv' yh'];         
        end

    end
    save(strcat('Matlab/',folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'), 't','tip_n', 'tip_r','base_n', 'base_r', 'X', 'Z', 'J', 'cmd', 'target', 'base_init', 'stage', 'depth', 'key', 'up', 'yp');
else    
    save(strcat('Matlab/',folder,'/',name,num2str(trial,'%2.2d'),extra,'.mat'), 't','tip_n', 'tip_r','base_n', 'base_r', 'X', 'Z', 'J', 'cmd', 'target', 'base_init', 'stage', 'depth', 'key', 'key_index');
end

function [horizontal, vertical]= get_needle_angles(quat)
    q = quaternion(quat);
    uz = quaternion(0, 0, 0, 1);
    v = compact(q*uz*q');

    horizontal = atan2(v(2),-v(3));                 % needle points in -Y direction
    vertical = atan2(v(4), sqrt(v(2)^2 + v(3)^2));
end