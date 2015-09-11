addpath(genpath('~/git/trajgen/matlab'));
clear
close all
clc

z_clear = 2; % z-height that is considered out of ground effect
r = 0.1; % m
bottom_height = 0*r;

% % For trials in motion
% transition_time = 5; % s
% bottom_distance = 1.5; % m
% bottom_velocity = .5; % m/s
% bottom_time = bottom_distance / bottom_velocity;
% xmax = 1.8; % Extent in the x-plane of the trajectory

transition_time = 5; % s
bottom_time = 3;
bottom_distance = 0; % m
bottom_velocity = bottom_distance / bottom_time; % m/s

xmax = 0; % Extent in the x-plane of the trajectory

% The time vector
t = [0, transition_time, transition_time + bottom_time, 2*transition_time + bottom_time];

% Return
t = [t, t(end) + t(2:end)];

% The gains
traj_kx = [3.7*ones(1,2), 8.0];
traj_kv = [2.4*ones(1,2), 3.0];

%% Waypoints
%
% Note that we only need to plan for x and z since the other dimensions are
% identically zero

d = 2;

% Trajectory Start
waypoints(1) = ZeroWaypoint(t(1), d);
waypoints(1).pos = [-xmax; z_clear];

% Start of bottom
waypoints(end+1) = ZeroWaypoint(t(2), d);
waypoints(end).pos = [-bottom_distance/2, bottom_height];
waypoints(end).vel(1) = bottom_velocity;

% End of bottom
waypoints(end+1) = ZeroWaypoint(t(3), d);
waypoints(end).pos = [bottom_distance/2, bottom_height];
waypoints(end).vel(1) = bottom_velocity;

% Trajectory End
waypoints(end+1) = ZeroWaypoint(t(4), d);
waypoints(end).pos = [xmax; z_clear];

%% Trajectory Return

% Start of bottom
waypoints(end+1) = ZeroWaypoint(t(5), d);
waypoints(end).pos = [bottom_distance/2; z_clear];
waypoints(end).vel(1) = -bottom_velocity;

% End of bottom
waypoints(end+1) = ZeroWaypoint(t(6), d);
waypoints(end).pos = [-bottom_distance/2; z_clear];
waypoints(end).vel(1) = -bottom_velocity;

% Trajectory End
waypoints(end+1) = ZeroWaypoint(t(7), d);
waypoints(end).pos = waypoints(1).pos;

options = {'ndim',2 ,'order',11, 'minderiv',[4,4], 'numerical', false};

% Generate the trajectory
traj = trajgen(waypoints, options);

%% Plotting

PlotTraj(traj)

ntraj = TrajEval(traj, 0:0.001:traj.keytimes(end));

figure();
plot(ntraj(:,1,1), ntraj(:,2,1), '.')
axis equal

% Some safety checks
max_acc = max(sqrt(ntraj(:,1,3).^2 + ntraj(:,2,3).^2));
fprintf('Max Acceleration: %2.2f m/s\n', max_acc);

%% Output to a csv

gains = [bsxfun(@times, ones(size(ntraj,1), 1), traj_kx), ...
         bsxfun(@times, ones(size(ntraj,1), 1), traj_kv)];

% Generate the array
zvec = zeros(size(ntraj,1), 1);
array = [...
    ntraj(:,1,1), zvec, ntraj(:,2,1), zvec, ...
    ntraj(:,1,2), zvec, ntraj(:,2,2), zvec, ...
    ntraj(:,1,3), zvec, ntraj(:,2,3), zvec, ...
    gains];

% Write to file and write dimensions on the first line
filename = 'traj.csv';
csvwrite(filename, array);
system(['printf "', num2str(size(array,1)), ',', '4,3,6\n',...
    '$( cat ', filename, ' )" > ', filename]);
save('workspace.mat')