clc; clear; close all;

% create and setup slam object 
slamObj = slam;

setup(slamObj);

% sim robot parameters
numBeams = 6;
angles = linspace(-pi, pi, numBeams);

% Start pose
pose = [1.0, 1.0, 0];

%  Truth Map
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

% Differential Drive Robot
wheelRadius = 0.1; % [m]   Radius of each wheel
trackWidth = 0.5; % [m]   Distance between left and right wheels
maxWheelSpeed = 100*2*pi/60; % [rad/s] Maximum wheel speed (100 RPM converted to rad/s)
vehicle = differentialDriveKinematics(VehicleInputs="VehicleSpeedHeadingRate");
vehicle.WheelSpeedRange = [-1 1]*maxWheelSpeed; 
vehicle.TrackWidth = trackWidth;
vehicle.WheelRadius = wheelRadius;

% Lidar Sensors
a = 0.15;                  % robot half-length [m]
b = 0.15;                  % robot half-width  [m]
o = 0.05; % sensor offset
maxRange = 4; % m

sensorOffset = [ a b;     % front-left
                  a (b-o);    % front-right
                  a (b -2*o)
                  a -(b -2*o)
                  a -(b-o);    % rear-left
                  a -b];   % rear-right

sensorAngle = deg2rad([60, 36, 12, -12, -36, -60]);

lidar1 = LidarSensor();
lidar2 = LidarSensor();
lidar3 = LidarSensor();
lidar4 = LidarSensor();
lidar5 = LidarSensor();
lidar6 = LidarSensor();


lidar1.mapName = mapTruth;
lidar1.sensorOffset = sensorOffset(1);
lidar1.scanAngles = sensorAngle(1);
lidar1.maxRange = maxRange;

lidar2.mapName = mapTruth;
lidar2.sensorOffset = sensorOffset(2);
lidar2.scanAngles = sensorAngle(2);
lidar2.maxRange = maxRange;

lidar3.mapName = mapTruth;
lidar3.sensorOffset = sensorOffset(3);
lidar3.scanAngles = sensorAngle(3);
lidar3.maxRange = maxRange;

lidar4.mapName = mapTruth;
lidar4.sensorOffset = sensorOffset(4);
lidar4.scanAngles = sensorAngle(4);
lidar4.maxRange = maxRange;

lidar5.mapName = mapTruth;
lidar5.sensorOffset = sensorOffset(5);
lidar5.scanAngles = sensorAngle(5);
lidar5.maxRange = maxRange;

lidar6.mapName = mapTruth;
lidar6.sensorOffset = sensorOffset(6);
lidar6.scanAngles = sensorAngle(6);
lidar6.maxRange = maxRange;

% Vector Field Histogram
VFH = controllerVFH();
VFH.NumAngularSectors = 36;
VFH.DistanceLimits = [0.05, 4];
VFH.RobotRadius = 0.2; % m
VFH.SafetyDistance = 0.3; % m
VFH.MinTurningRadius = 0.5; % m
VFH.TargetDirectionWeight = 5;
VFH.CurrentDirectionWeight = 2;
VFH.PreviousDirectionWeight = 2;
VFH.HistogramThresholds = [3, 10];
VFH.UseLidarScan = false;

% Pure Pursuit
vMax = 0.5;   % your desired speed
vMin = 0.1;
desiredSpeed = 0.5; % m/s
PP = controllerPurePursuit();
PP.DesiredLinearVelocity = desiredSpeed;
PP.Waypoints = [1.5 1.5]; % center of field

% Calculate maximum angular velocity based on vehicle physical limits.
% For a differential drive vehicle, maximum angular velocity occurs when
% the wheels rotate in opposite directions at maximum speed.
maxAngularVelocity = 3;                   % rad/s

lookaheadTime = 2;                                                             % [seconds]
PP.LookaheadDistance = 0.8; % [meters]


% Simulation loop 
% Define simulation parameters
sampleTime = 0.1;                        % Time step for simulation [seconds]
maxSimTime = 30;                         % Maximum simulation time [seconds]
simSteps = ceil(maxSimTime/sampleTime);  % Total number of simulation steps
timeVec = (0:simSteps-1)*sampleTime;

initialPose = [1 1 0];                   % Initial pose: [x y theta]
currentPose = initialPose;               % Set current pose to initial pose
poses = zeros(simSteps, 3);              % Preallocate array to record robot trajectory
% Initialize arrays to record velocity commands
vCmds = zeros(simSteps,1);               % Linear velocity commands
wCmds = zeros(simSteps,1);               % Angular velocity commands
% Reset the pure pursuit controller to clear any previous states from
% previous runs
reset(PP);


for idx = 1:simSteps

   % Simulate Lidar
    pts = rayIntersection(mapTruth, currentPose, angles, maxRange);
    dx = pts(:,1) - currentPose(1);
    dy = pts(:,2) - currentPose(2);
    ranges = sqrt(dx.^2 + dy.^2)';
    ranges(isnan(ranges)) = maxRange;

    % SLAM + planning
    [map, goalXY, path, pathLength, hasFrontier, mapUpdated, frontierXY] = ...
        step(slamObj, ranges, angles, currentPose);

    % Pull out non zero way points
    if (pathLength > 0)
        PP.Waypoints = path(1:pathLength, :);
    end

    % Controllers

    % Pure Pursuit
    [vCmd,kappaCmd] = PP(currentPose);
    vCmd = vMax * (1 - 0.5*abs(kappaCmd));  
    vCmd = max(vMin, min(vCmd, vMax));
    wPP = kappaCmd * vCmd;

    % VFH
    if pathLength > 1
        nextPoint = path(2,:);
    else
        nextPoint = goalXY;
    end
    
    desiredHeading = atan2(nextPoint(2)-currentPose(2), ...
                           nextPoint(1)-currentPose(1));
    steeringDirection = VFH(ranges, angles, desiredHeading);

    headingError = wrapToPi(steeringDirection - currentPose(3));
    wVFH = 2.0 * headingError;

    % blend both
    alpha = 0.1;
    wCmd = alpha*wVFH + (1-alpha)*wPP;

    % clamp output
    wCmd = max(min(wCmd, maxAngularVelocity), -maxAngularVelocity);

    % update sim pose
    vel = derivative(vehicle,currentPose,[vCmd wCmd]);
    currentPose = currentPose + vel'*sampleTime;
    poses(idx,:) = currentPose;

   
    fprintf("Step %d | Frontiers: %d | PathLen: %d | Steering Direction: %d\n", ...
        idx, hasFrontier, pathLength, steeringDirection);
    pause(0.2);

    % Ploting
    if mod(idx,1) == 0
        figure(1); clf;

        subplot(1,2,1)
        show(mapTruth);
        hold on;
        plot(currentPose(1), currentPose(2), 'ro');
        title('Ground Truth')

        subplot(1,2,2)
        axis equal tight;
        title('SLAM Map')
        show(map);
        hold on;
        scatter(frontierXY(:,1), frontierXY(:,2), 50, 'r', 'filled'); % frontiers
        plot(goalXY(1), goalXY(2), 'g*', 'MarkerSize', 12);           % selected goal
        plot(currentPose(1), currentPose(2), 'bo');                   % robot
        
        % plot planned patg
        if ~isempty(path)
            plot(PP.Waypoints(:,1), PP.Waypoints(:,2), 'b-', 'LineWidth', 2);
        end
        drawnow;
    end

end