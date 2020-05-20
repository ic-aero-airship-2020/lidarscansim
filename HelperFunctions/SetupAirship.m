function [diffDrive,controller,frontSensor,leftSensor,rightSensor] = SetupAirship(thrustVectorDist,maxThrust,minThrust,cruiseSpeed,maxAngularVelocity,sensorMaxRange,sensorFieldOfVision)
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
end