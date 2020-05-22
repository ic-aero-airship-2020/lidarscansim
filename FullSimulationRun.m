%% Cleaning environment
clear
clc
close all

%% Step 0. Setting up the simualation
addpath('./HelperFunctions/');
addpath('./3DModel/');
% Choosing the map to run the simulation on
% ######## Choose map complexity ########
% 1 - simple map complexity
% 2 - complex map complexity
% 3 - imperial map complexity
complexity = 2
% #######################################
[manualFig,referenceMap,manualPath,frameSize,bwImage] = SetupSimulationEnvironment(complexity);

% Configuring airship and its kinematics
% ######## Change these when the actual build is defined ########
thrustVectorDist = 0.2;
maxThrust = Inf;
minThrust = -Inf;
cruiseSpeed = 2; % m/s
maxAngularVelocity = 3; % rads/s

sensorMaxRange = 10; % m
sensorFieldOfVision = 20/180*pi; % rad
leftSensorOffset = pi*2/3; % rad
rightSensorOffset = -pi*2/3; % rad
% ###############################################################
[diffDrive,controller,frontSensor,leftSensor,rightSensor] = SetupAirship(thrustVectorDist,maxThrust,minThrust,cruiseSpeed,maxAngularVelocity,sensorMaxRange,sensorFieldOfVision);

[virtualWorld,virtualWorldFigure] = Setup3DSimulationEnvironment(referenceMap,bwImage,[manualPath(1,2) manualPath(1,1)]);
simOut = sim('Closed_Loop_Model.slx');
yaw = simOut.logs(:,1);
x   = simOut.logs(:,2);
y   = simOut.logs(:,3);
z   = simOut.logs(:,4);

%% Step 1. Manual flight simulation to generate a known map
% blank map to be populated
mapXDim = referenceMap.XWorldLimits(2);
mapYDim = referenceMap.YWorldLimits(2);
generatedMap = binaryOccupancyMap(mapXDim,mapYDim,10);

% visualising the maps
figure(manualFig)
subplot(1,2,2)
show(generatedMap)

h = findobj(gcf,'type','axes');
refFig = h(2);
genFig = h(1);

% emulating the motion
controller.Waypoints = manualPath;

initPose = [manualPath(1,1) manualPath(1,2), pi/2];
goal = [manualPath(end,1) manualPath(end,2)]';
poses(:,1) = initPose';
[lidarData,generatedMap] = flyAirship(diffDrive,controller,initPose,goal,referenceMap,generatedMap,refFig,genFig,frontSensor,leftSensor,rightSensor,leftSensorOffset,rightSensorOffset,frameSize);

%% Step 2. Plan a initial flight path
% Set the start and goal poses
switch complexity
    case 1 
        start = [24.5, 2.35, pi/2];
        goal = [3.65, 3.55, 0];
    case 2
        start = [46.5, 35.5 pi/2];
%         goal = [13.5, 22.5, 0];
        goal = [46.05, 5.75, 0];
    case 3
        start = [185.1, 112.6, pi];
        goal = [66.44, 24.44, pi];
    otherwise
        start = [24.5, 2.35, pi/2];
        goal = [3.65, 3.55, 0];
end

minTurningRadius = 0.4;
validationDistance = 0.05; % m
maxAirshipHalfWidth = diffDrive.TrackWidth/2;

[plannedPoses,planFig] = PlanPath(generatedMap,start,goal,minTurningRadius,maxAirshipHalfWidth,validationDistance);

%% Step 3. Autonomous flying (Simultaneous Map and Plan)
plannedFig = figure("Name", "Planned Path",'WindowState', 'maximized');
subplot(1,2,1)
show(referenceMap);
hold on
plot(plannedPoses(:,1), plannedPoses(:,2),"r-")
hold off
title("Reference Map")
subplot(1,2,2)
show(generatedMap);

h = findobj(plannedFig,'type','axes');
refFig = h(2);
genFig = h(1);


% Add iterative process of solving of updating the current map and
% recalculating the path to be taken
controller.Waypoints = plannedPoses;

initPose = [plannedPoses(1,1) plannedPoses(1,2), pi/2];
goal = [plannedPoses(end,1) plannedPoses(end,2)]';
poses(:,1) = initPose';
[lidarData,generatedMap] = flyAirship(diffDrive,controller,initPose,goal,referenceMap,generatedMap,refFig,genFig,frontSensor,leftSensor,rightSensor,leftSensorOffset,rightSensorOffset,frameSize);