clear
clc
close all

%% Testing on selected map
% 1 - simple map complexity
% 2 - complex map complexity
% 3 - imperial map complexity
complexity = 1;
[manualFig,referenceMap,manualPath,frameSize] = SetupSimulationEnvironment(complexity);


%% Generating lidar data
% blank map to be populated
mapXDim = referenceMap.XWorldLimits(2);
mapYDim = referenceMap.YWorldLimits(2);
generatedMap = binaryOccupancyMap(mapXDim,mapYDim,10);

figure(manualFig)
subplot(1,2,2)
show(generatedMap)

% Defining airship
% modelling the kinematics of the airship as a differential drive
% kinematics. i.e. left and right thrust present

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

h = findobj(manualFig,'type','axes');
refFig = h(2);
genFig = h(1);

% Simulates the manual flying of the drone initially to collect data

controller.Waypoints = manualPath;

initPose = [manualPath(1,1) manualPath(1,2), pi/2];
goal = [manualPath(end,1) manualPath(end,2)]';
poses(:,1) = initPose';
[lidarData,generatedMap] = flyAirship(diffDrive,controller,initPose,goal,referenceMap,generatedMap,refFig,genFig,frontSensor,leftSensor,rightSensor,leftSensorOffset,rightSensorOffset,frameSize);
