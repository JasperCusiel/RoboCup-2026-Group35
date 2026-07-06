clc; clear; close all;

%% Create Truth Map
res = 10;  % cells per metre
mapTruth = binaryOccupancyMap(3, 5, res);

gridSize = mapTruth.GridSize;
grid = zeros(gridSize);
grid(1, :)   = 1;   % bottom wall
grid(end, :) = 1;   % top wall
grid(:, 1)   = 1;   % left wall
grid(:, end) = 1;   % right wall

grid(1:8, 15) = 1;   % internal vertical line

grid(20, 11:19) = 1;   % internal horizontal line

grid(29:35, 15) = 1;   % internal vertical line
grid(35, 12:18) = 1;   % internal horizontal line

grid(44:50, 15) = 1;   % internal vertical line
setOccupancy(mapTruth, grid);

%% Create and setup slam object 
slamObj = slam;
setup(slamObj);


%% Robot Setup

% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.2;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:60;        % Time array

% Initial conditions
initPose = [0.5;4.5;-pi / 2];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;


%% Lidar Sensors
lidar = LidarSensor;
lidar.mapName = 'map';
lidar.sensorOffset = [0,0];
lidar.scanAngles = deg2rad(linspace(-60,60,78)); 
lidar.maxRange = 4;


%% Visualizer
viz = Visualizer2D;
viz.mapName = 'mapTruth';
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

% Create waypoints
%waypoints = [initPose(1:2)'; 2.5 1; 1 4; 2.8 4.5; 2.5 1; initPose(1:2)'];
waypoints = [initPose(1:2)'];

% Pure Pursuit Controller
maxVelocity = 0.4;
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0;
controller.DesiredLinearVelocity = maxVelocity;
controller.MaxAngularVelocity = 3;

r = rateControl(1/sampleTime);

% speed PD
prevError = 0;

% escape logic
prevSteerDir = 0;


for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);

    % SLAM + planning
    [map, goalXY, path, pathLength, hasFrontier, mapUpdated, frontierXY] = ...
        step(slamObj, ranges, lidar.scanAngles, curPose);

    % Pull out non zero way points
    % if (pathLength > 0)
    %     controller.Waypoints = path(1:pathLength, :);
    % end
    controller.Waypoints = goalXY;

    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges, lidar.scanAngles, targetDir); 

    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.2*steerDir;
    end

    if isnan(steerDir)
        if isnan(prevSteerDir) || prevSteerDir == 0
            turnDir = 1; % default direction
        else
            turnDir = sign(prevSteerDir);
        end
        wRef = 0.5 * turnDir;
        controller.DesiredLinearVelocity = 0;
    else

        % speed PD
        dt = sampleTime;
        
        err = abs(angdiff(targetDir, steerDir));
        
        dErr = (err - prevError) / dt;
        
        Kp = 0.3;
        Kd = 0.1;
        
        speedScale = 1 - (Kp * err + Kd * dErr);
        
        speedScale = min(max(speedScale, 0.1), 1.0);
        
        prevError = err;
        controller.DesiredLinearVelocity = maxVelocity * speedScale;
    
    end

    prevSteerDir = steerDir;

    vRef = controller.DesiredLinearVelocity;

    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration ste
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx), controller.Waypoints, ranges);

    figure(3); clf;
    show(vfh);

    figure(2); clf;
    show(map);
    hold on;
    scatter(frontierXY(:,1), frontierXY(:,2), 50, 'r', 'filled'); % frontiers
    plot(goalXY(1), goalXY(2), 'g*', 'MarkerSize', 12);           % selected goal
    plot(curPose(1), curPose(2), 'bo');                   % robot
    
    % plot planned patg
    if ~isempty(path)
        plot(controller.Waypoints(:,1), controller.Waypoints(:,2), 'b-', 'LineWidth', 2);
    end
    hold on;
    drawnow;
    waitfor(r);
end
