clear
close all
clc

transition_time = 5; % s
bottom_distance = 2; % m
bottom_velocity = .5; % m/s

% The time vector
bottom_time = bottom_distance / bottom_velocity;
t = [0, transition_time, transition_time + bottom_time, 2*transition_time + bottom_time];

%% Waypoints
% 
% Note that we only need to plan for x and z since the other dimensions are
% identically zero

d = 2;

% Trajectory Start
waypoints(1) = ZeroWaypoint(t(1), d);
waypoints(1).pos = [-2.5; 1];

% Start of bottom
waypoints(2) = ZeroWaypoint(t(2), d);
waypoints(2).pos(1) = -bottom_distance/2;
waypoints(2).vel(1) = bottom_velocity;

% End of bottom
waypoints(3) = ZeroWaypoint(t(3), d);
waypoints(3).pos(1) = bottom_distance/2;
waypoints(3).vel(1) = bottom_velocity;

% Trajectory End
waypoints(4) = ZeroWaypoint(t(4), d);
waypoints(4).pos = [2.5; 1];
  
options = {'ndim',2 ,'order',7, 'minderiv',[4,4]};

% Generate the trajectory
traj = trajgen(waypoints, options);

%% Plotting

% PlotTraj(traj)

ntraj = TrajEval(traj, 0:0.001:traj.keytimes(end));

plot(ntraj(:,1,1), ntraj(:,2,1), '.')
axis equal

% Some safety checks
max_acc = max(sqrt(ntraj(:,1,3).^2 + ntraj(:,2,3).^2));
fprintf('Max Acceleration: %2.2f m/s\n', max_acc);

%% Output to a csv

% Generate the array
zvec = zeros(size(ntraj,1), 1);
array = [...
    ntraj(:,1,1), zvec, ntraj(:,2,1), zvec, ...
    ntraj(:,1,2), zvec, ntraj(:,2,2), zvec, ...
    ntraj(:,1,3), zvec, ntraj(:,2,3), zvec];

% Write to file and write dimensions on the first line
filename = 'traj.csv';
csvwrite(filename, array);
system(['printf "', num2str(size(array,1)), ',', '4,3\n',...
    '$( cat ', filename, ' )" > ', filename]);
