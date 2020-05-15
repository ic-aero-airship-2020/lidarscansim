clc;
clear;
close all;

%% Testing on basic map
load exampleMaps.mat
referenceMap = binaryOccupancyMap(simpleMap,1);

%% Generating Map of CAGB and Skem
actualLayout = imread('Map/Actual Layout.png');
simplifiedLayout = imread('Map/Simplified 2D Layout.png');
bwImage = simplifiedLayout > 0;
% referenceMap = binaryOccupancyMap(bwImage,8);


figure('Name','Comparing layout');
subplot(2,2,1);
imshow(actualLayout);
title("Actual Layout");
subplot(2,2,2);
imshow(imcomplement(bwImage));
title("Simplified Layout");
subplot(2,2,3);
show(referenceMap)

refFig = figure('Name','Reference Map');
show(referenceMap)


%% Generating lidar data
% blank map to be populated
mapXDim = referenceMap.XWorldLimits(2);
mapYDim = referenceMap.YWorldLimits(2);
generatedMap = binaryOccupancyMap(mapXDim,mapYDim,10);
genFig = figure('Name','Generated Map');
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

% driver initialisation
wheelSpeedRange = [minThrust, maxThrust];
diffDrive = differentialDriveKinematics(...
    "VehicleInputs","VehicleSpeedHeadingRate",...
    "WheelSpeedRange",wheelSpeedRange,...
    "TrackWidth",thrustVectorDist);
controller = controllerPurePursuit(...
    'DesiredLinearVelocity',cruiseSpeed,...
    'MaxAngularVelocity',maxAngularVelocity);

% sensor initialisation
frontSensor = rangeSensor;
frontSensor.Range = [0,sensorMaxRange];
frontSensor.HorizontalAngle = [-sensorFieldOfVision/2, sensorFieldOfVision/2];
leftSensor = rangeSensor;
leftSensor.Range = [0,sensorMaxRange];
leftSensor.HorizontalAngle = [-sensorFieldOfVision/2, sensorFieldOfVision/2];
rightSensor = rangeSensor;
rightSensor.Range = [0,sensorMaxRange];
rightSensor.HorizontalAngle = [-sensorFieldOfVision/2, sensorFieldOfVision/2];

% Simulates the manual flying of the drone initially to collect data
path = [186.4, 108.2;
        174.7, 107.4;
        173.5, 121.9;
        165.2, 122.7;
        165.2, 112.8;
        160.1, 115.3;
        154.1, 115.3;
        150.3, 110.6;
        148.4, 114.1;
        137.3, 114.6;
        136.8, 118.3;
        144.4, 118.1;
        145.3, 114.8;
        165.2, 112.8;
        164.3, 102.9;
        167.1, 89.06;
        159.1, 90.69;
        166.6, 86.69;
        165.8, 79.06;
        157.2, 78.19;
        158.8, 67.31;
        158.8, 53.56;
        158.2, 27.44;
        145, 27.06;
        144.5, 28.81;
        129.8, 28.44;
        126.5, 23.19;
        113.8, 22.69;
        113.1, 29.19;
        78.44, 27.81;
        75.19, 27.69;
        74.94, 24.69;
        66.44, 24.44;];

path = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
    
% Manual driving path plotted
figure(refFig)
hold on
plot(path(:,1),path(:,2), 'o-');
legend("Manual driving path")
hold off

controller.Waypoints = path;

initPose = [path(1,1) path(1,2), pi/2];
goal = [path(end,1) path(end,2)]';
poses(:,1) = initPose';
onSimumation = false; % To display simulation, change value to True
lidarData = flyAirship(diffDrive,controller,initPose,goal,referenceMap,generatedMap,refFig,genFig,frontSensor,leftSensor,rightSensor,leftSensorOffset,rightSensorOffset);


%% Creating visual model of the airship
% 
% 
% 
% % to account for the radius of the airship we add a buffer to the walls
% airship_max_radius = 1.0; % in m
% map_inflated = copy(referenceMap);
% inflate(map_inflated,airship_max_radius);
% figure(3)
% show(map_inflated)

