% clc; clear; 

% figure setup
% --- Setup once ---
figure(2); clf;
show(map);           % draw once to create the image object
colormap(gca, 'gray');
hold on;
hImg = findobj(gca, 'Type', 'image');   % grab the occupancy image handle
hFrontiers = scatter(nan, nan, 50, 'r', 'filled');
hGoal = plot(nan, nan, 'g*', 'MarkerSize', 12);
hRobot = plot(nan, nan, 'bo');
hold off;

%% Create Arena Map
res = 20;  % cells per metre
inflate_by = 2;
arena_map = binaryOccupancyMap(3, 5, res);

gridSize = arena_map.GridSize;
grid = zeros(gridSize);
grid(1, :)   = 1;   % bottom wall
grid(end, :) = 1;   % top wall
grid(:, 1)   = 1;   % left wall
grid(:, end) = 1;   % right wall

grid(1:(0.8*res), (1.5 * res)) = 1;   % internal vertical line

grid((2*res), (1.1 * res):(1.9 * res)) = 1;   % internal horizontal line

grid((2.9 * res):(3.5 * res), (1.5 * res)) = 1;   % internal vertical line
grid((3.5 * res), (1.2 * res):(1.8*res)) = 1;   % internal horizontal line

grid((4.4 * res):(5.0 * res), (1.5 * res)) = 1;   % internal vertical line
setOccupancy(arena_map, grid);

% Create and setup slam object 
slamObj = slam;
setup(slamObj);


% Robot Setup
wheel_radius = 0.1;                        % Wheel radius [m]
wheel_base = 0.2;                        % Wheelbase [m]
dd = DifferentialDrive(wheel_radius,wheel_base);

% Simulation Setup
sample_time = 0.1;              % Sample time [s]
sim_duration = 60;
starting_heading = -pi;
%initial_pose = [0.5;4.5;starting_heading];
initial_pose = [2.5;4.5;starting_heading];

time_vector = 0:sample_time:sim_duration;        % Time array

% Initial conditions
pose = zeros(3,numel(time_vector));   % Pose matrix
pose(:,1) = initial_pose;


% Lidar Sensor Setup
lidar = LidarSensor;
lidar.mapName = 'arena_map';
lidar.sensorOffset = [0,0];
lidar.scanAngles = deg2rad(linspace(-60,60,78)); 
lidar.maxRange = 4;


% Visualizer
viz = Visualizer2D;
viz.mapName = 'arena_map';
viz.hasWaypoints = true;
viz.robotRadius = 0.1;
attachLidarSensor(viz, lidar);

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 1.5];
vfh.NumAngularSectors = 180;
vfh.HistogramThresholds = [6 8];
vfh.RobotRadius = 0.1;
vfh.SafetyDistance = 0.1;
vfh.MinTurningRadius = 0.1;

% Waypoints
% waypoints = [initial_pose(1:2)'; 2.5 1; 1 4; 2.8 4.5; 2.5 1; initial_pose(1:2)'];
waypoints = initial_pose(1:2)';

% Pure Pursuit Controller
maxVelocity = 0.2;
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = maxVelocity;
controller.MaxAngularVelocity = 3;

r = rateControl(1/sample_time);

% Speed PD
prevError = 0;
Kp = 0.2;
Kd = 0.05;

% escape logic
prevSteerDir = 0;


for idx = 2:numel(time_vector) 
  
    current_pose = pose(:,idx-1);

    ranges = lidar(current_pose);

    % SLAM + frontier picking
    [map, goalXY, path, pathLength, hasFrontier, frontierXY] = ...
        step(slamObj, ranges, lidar.scanAngles, current_pose);
    display(pathLength);
    % Set pure persuit target
    if pathLength > 0
        controller.Waypoints = path(1:pathLength, :);
    end

    % Run the path following and obstacle avoidance algorithms

    [~, wRef, lookAheadPt] = controller(current_pose);

    targetDir = atan2(lookAheadPt(2)-current_pose(2),lookAheadPt(1)-current_pose(1)) - current_pose(3);

    steerDir = vfh(ranges, lidar.scanAngles, targetDir); 
    
    % If valid steering direction
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.6*steerDir;
    end
    
    % If VFH cant find a valid direction to steer
    if isnan(steerDir)
        % Turn on spot in previous steering direction
        if isnan(prevSteerDir) || prevSteerDir == 0
            turnDir = 1; % default direction
        else
            turnDir = sign(prevSteerDir);
        end

        % Stop and spin on the spot
        wRef = 0.5 * turnDir;
        controller.DesiredLinearVelocity = 0;
    else

        % PD control for speed based on how strong VFH is steering us away
        % from obsticals.
        err = abs(angdiff(targetDir, steerDir));
        
        dErr = (err - prevError) / sample_time;
        
        speed_scale = 1 - (Kp * err + Kd * dErr);
        
        speed_scale = min(max(speed_scale, 0.1), 1.0);
        
        prevError = err;
        controller.DesiredLinearVelocity = maxVelocity * speed_scale;
    
    end

    prevSteerDir = steerDir;

    vRef = controller.DesiredLinearVelocity;

    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,current_pose);  % Convert from body to world
    
    % Perform forward discrete integration ste
    pose(:,idx) = current_pose + vel * sample_time; 
    
    % Update visualization
    viz(pose(:,idx), controller.Waypoints, ranges);
    
    % Show VFH polar plots
    % if mod(idx, 3) == 0
    %     figure(3);
    %     show(vfh);
    % end
    
    % Show occupancy map
    set(hImg, 'CData', occupancyMatrix(map));   % just refresh the grid data
    set(hFrontiers, 'XData', frontierXY(:,1), 'YData', frontierXY(:,2));
    set(hGoal, 'XData', goalXY(1), 'YData', goalXY(2));
    set(hRobot, 'XData', current_pose(1), 'YData', current_pose(2));
    drawnow limitrate;
    waitfor(r);
end
