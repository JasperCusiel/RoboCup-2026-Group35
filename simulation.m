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
grid(10:30, 25) = 1;   % internal vertical line
grid(30, 10:20) = 1;   % internal horizontal line
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
tVec = 0:sampleTime:45;        % Time array

% Initial conditions
initPose = [1;1;0];            % Initial pose (x y theta)
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
attachLidarSensor(viz, lidar);

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 1.5];
vfh.NumAngularSectors = 180;
vfh.HistogramThresholds = [4 5];
vfh.RobotRadius = 0.1;
vfh.SafetyDistance = 0.01;
vfh.MinTurningRadius = 0.05;

% Create waypoints
waypoints = [initPose(1:2)'; 2.5 1; 1 4; 2.8 4.5; 2.5 1; initPose(1:2)'];

% Pure Pursuit Controller
maxVelocity = 0.25;
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = maxVelocity;
controller.MaxAngularVelocity = 1.5;

r = rateControl(1/sampleTime);

for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);

    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges, lidar.scanAngles, targetDir); 

    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.2*steerDir;
    end
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration ste
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx), waypoints, ranges);
    figure(3); clf;
    show(vfh);
    hold on;
    drawnow;
    waitfor(r);
end
