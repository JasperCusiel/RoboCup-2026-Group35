classdef slam < matlab.System
    % untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        maxRange = 5;  % meters
        resolution = 10;  % cells per meter
        loopClosureThreshold = 360;
        loopClosureSearchRadius = 8;
        rebuildEveryN = 10; % rebuild occupancy grid ever N scans
        arenaWidth = 3;
        arenaHeight = 5;
        maxNumScans = 500;
        maxPathLength = 200;
        maxFrontierCells = 1000;
        maxClusters = 50;
        minClusterSize = 5;
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        SlamObj             % the underlying lidarSLAM object
        Correction          % [x y theta]: dead-reckoning frame -> world frame
        PrevDeadReconingPose          % dead-reckoning pose at the previous accepted scan
        Map             % fixed size to the arena
        ScansSinceRebuild
        CachedGoalXY
        CachedPath
        CachedPathLength
        CachedHasFrontier
        
    end

    methods (Access = protected)
        function setupImpl(obj)
            %#codegen
            % Perform one-time calculations, such as computing constants
            obj.SlamObj = lidarSLAM(obj.resolution,obj.maxRange, obj.maxNumScans);
            obj.SlamObj.LoopClosureThreshold = obj.loopClosureThreshold;
            obj.SlamObj.LoopClosureSearchRadius = obj.loopClosureSearchRadius;

            obj.Map = occupancyMap(obj.arenaWidth, obj.arenaHeight, obj.resolution);


            obj.Correction        = [0 0 0];
            obj.PrevDeadReconingPose        = [0 0 0];
            obj.ScansSinceRebuild = 0;
            obj.CachedGoalXY      = [0 0];
            obj.CachedPath        = zeros(obj.maxPathLength, 2);
            obj.CachedPathLength  = 0;
            obj.CachedHasFrontier = false;

        end

        function [map, goalXY, path, pathLength, hasFrontier, mapUpdated, frontierXY] = stepImpl(obj,ranges, angles, deadReckoningPose)
           %#codegen
            scan = lidarScan(ranges, angles);
           % range, angles -> new lidar scan
           % deadReckoningPose -> from IMU + optical flow measurements
           deadReckoningPose = reshape(deadReckoningPose, 1, 3);

           relativePoseEstimate = composeSE2(obj, invertSE2(obj, obj.PrevDeadReconingPose), deadReckoningPose);
           obj.PrevDeadReconingPose = deadReckoningPose;

           [isAccepted, ~, optInfo] = addScan(obj.SlamObj, scan, relativePoseEstimate);

           mapUpdated = false;
           frontierXY = [0 0];
           % correctedPoseNow = composeSE2(obj, obj.Correction, deadReckoningPose);


           if isAccepted
               [~, poses] = scansAndPoses(obj.SlamObj);
               slamPoseNow = poses(end, :);

               obj.Correction = composeSE2(obj, slamPoseNow, invertSE2(obj, deadReckoningPose));

               insertRay(obj.Map, deadReckoningPose, scan, obj.maxRange);

               obj.ScansSinceRebuild = obj.ScansSinceRebuild + 1;
               mapUpdated = optInfo.IsPerformed || obj.ScansSinceRebuild >= obj.rebuildEveryN;
           end

           if mapUpdated
                   obj.ScansSinceRebuild = 0; % Reset the rebuild counter
                   [goalXYNow, hasFrontierNow, frontierXY] = obj.pickFrontierGoal(obj.Map, deadReckoningPose(1:2));
                   
                   obj.CachedHasFrontier = hasFrontierNow;
                   if hasFrontierNow
                       obj.CachedGoalXY = goalXYNow;
                       rawPath = obj.planPathToGoal(obj.Map, deadReckoningPose(1:2), goalXYNow);
                       n = min(size(rawPath, 1), obj.maxPathLength);
                       obj.CachedPath = zeros(obj.maxPathLength, 2);
                       obj.CachedPath(1:n, :) = rawPath(1:n, :);
                       obj.CachedPathLength = n;
                   else
                       obj.CachedPathLength = 0;
                   end

           end

           % correctedPose = correctedPoseNow;
           map       = obj.Map; 
           goalXY        = obj.CachedGoalXY;
           path          = obj.CachedPath;
           pathLength    = obj.CachedPathLength;
           hasFrontier   = obj.CachedHasFrontier;

           

        end

        function resetImpl(obj)
            %#codegen
            % Initialize / reset internal properties
            setupImpl(obj);
        end
    end

    methods 
        function pose = getCorrectedPose(obj, deadReckoningPoseNow)
            %#codegen
            pose = composeSE2(obj, obj.Correction, deadReckoningPoseNow);
        end
    end

    methods (Access = private)

        function p = composeSE2(~, poseA, poseB)
            %#codegen
            c = cos(poseA(3));
            s = sin(poseA(3));
            p = [poseA(1) + c*poseB(1) - s*poseB(2), poseA(2) + s*poseB(1) + c*poseB(2), wrapToPiLocal(-poseA(3))];
        end

        function p = invertSE2(~, poseA)
            %#codegen
            c = cos(poseA(3));
            s = sin(poseA(3));
            p = [-c*poseA(1) - s*poseA(2), s*poseA(1) - c*poseA(2), wrapToPiLocal(-poseA(3))];
        end

        function [goalXY, hasFrontier, frontierXY] = pickFrontierGoal(obj, map, robotPose)
            %#codegen
            frontierXY = [0 0];
            
            occTern = occupancyMatrix(map, 'ternary');
            fMask = obj.frontierMask(occTern);
            
            [centroidsRC, sizes, numClusters] = obj.clusterFrontierCells(fMask);
            
            goalXY = zeros(1,2);
            hasFrontier = false;
            
            if numClusters == 0
                return
            end
            
            centroidsRC = centroidsRC(:,1:2);
            
            centroidsValid = zeros(obj.maxClusters,2);
            sizesValid = zeros(obj.maxClusters,1);
            
            nValid = 0;
            
            for i = 1:numClusters
                if sizes(i) >= obj.minClusterSize
                    nValid = nValid + 1;
                    centroidsValid(nValid,:) = centroidsRC(i,:);
                    sizesValid(nValid) = sizes(i);
                end
            end
            
            if nValid == 0
                return
            end
            
            centroidsXY = zeros(nValid,2);
            
            for i = 1:nValid
                rc = round(centroidsValid(i,:));
                centroidsXY(i,:) = grid2world(map, rc);
            end
            
            goalXY = obj.selectFrontierGoal( ...
                centroidsXY, ...
                sizesValid(1:nValid), ...
                robotPose(1:2), nValid);
            
            hasFrontier = true;
            
            frontierXY = centroidsXY(1:nValid,:);
            end
    end
    
    methods (Access = private, Static)
        function fMask = frontierMask(occTern)
            %#codegen
            [rows, cols] = size(occTern);
            
            free    = (occTern == 0);
            unknown = (occTern == -1);
            
            unkDilated = unknown;
            
            shifts = int32([-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1]);
            
            for k = 1:8
                dr = shifts(k,1);
                dc = shifts(k,2);
            
                shifted = false(rows, cols);
            
                rStart = max(1, 1-dr);
                rEnd   = min(rows, rows-dr);
                cStart = max(1, 1-dc);
                cEnd   = min(cols, cols-dc);
            
                for r = rStart:rEnd
                    for c = cStart:cEnd
                        shifted(r+dr, c+dc) = unknown(r,c);
                    end
                end
            
                unkDilated = unkDilated | shifted;
            end
            
            fMask = free & unkDilated;
        end
             
        function [centroidsRC, sizes, numClusters] = clusterFrontierCells(mask)
            %#codegen
            [rows, cols] = size(mask);
            
            visited = false(rows, cols);
            
            MAX_CELLS = rows * cols;
            MAX_CLUSTERS = 1000;
            
            centroidsRC = zeros(MAX_CLUSTERS, 2);
            sizes       = zeros(MAX_CLUSTERS, 1);
            numClusters = 0;
            
            stack = zeros(MAX_CELLS, 2);
            comp  = zeros(MAX_CELLS, 2);
            
            for r = 1:rows
                for c = 1:cols
            
                    if mask(r,c) && ~visited(r,c)
            
                        numClusters = numClusters + 1;
            
                        top = 1;
                        stack(1,:) = [r c];
            
                        compSize = 1;
                        comp(1,:) = [r c];
            
                        visited(r,c) = true;
            
                        % depth first search
                        while top > 0
            
                            cur = stack(top,:);
                            top = top - 1;
            
                            for dr = -1:1
                                for dc = -1:1
            
                                    if dr == 0 && dc == 0
                                        continue;
                                    end
            
                                    nr = cur(1) + dr;
                                    nc = cur(2) + dc;
            
                                    if nr>=1 && nr<=rows && nc>=1 && nc<=cols ...
                                            && mask(nr,nc) && ~visited(nr,nc)
            
                                        visited(nr,nc) = true;
            
                                        % push
                                        top = top + 1;
                                        stack(top,:) = [nr nc];
            
                                        compSize = compSize + 1;
                                        comp(compSize,:) = [nr nc];
                                    end
                                end
                            end
                        end
            
                        % compute centroid
                        sumR = 0;
                        sumC = 0;
            
                        for i = 1:compSize
                            sumR = sumR + comp(i,1);
                            sumC = sumC + comp(i,2);
                        end
            
                        centroidsRC(numClusters,1) = sumR / compSize;
                        centroidsRC(numClusters,2) = sumC / compSize;
                        sizes(numClusters) = compSize;
                    end
                end
            end
        end
 
        function goalXY = selectFrontierGoal(centroidsXY, sizes, robotXY, numClusters)
                %#codegen
                
                bestIdx = 1;
                bestUtility = -1;
                
                for i = 1:numClusters
                    dx = centroidsXY(i,1) - robotXY(1);
                    dy = centroidsXY(i,2) - robotXY(2);
                
                    dist = sqrt(dx*dx + dy*dy);
                
                    utility = sizes(i) / (dist + 0.1);
                
                    if utility > bestUtility
                        bestUtility = utility;
                        bestIdx = i;
                    end
                end
                
                goalXY = centroidsXY(bestIdx,:);
        end
 
        function path = planPathToGoal(map, startXY, goalXY)
            planner = plannerAStarGrid(map);
            path = plan(planner, startXY, goalXY, 'world');
        end
    end
end

 function a = wrapToPiLocal(a)
            %#codegen
            a = mod(a + pi, 2*pi) - pi;
 end