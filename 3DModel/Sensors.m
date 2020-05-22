%define sensor types

%Sensor.SensorName.
% -Range.
    % - day = day time range
    % - night = night time range
% - FoV = field of vision
% - Size = [width length]
% - Centre = [where is assumed possition of sensor on breakout]
% - Shape  = [coordinate set 1; .......; coordinate set 4] (starts from top left rotates clockwise)


%sensor 1  = ToF Sensor

Sensor.ToFSensor.Range.day   = 2;
Sensor.ToFSensor.Range.night = 3.6;
Sensor.ToFSensor.FoV  = 27;
Sensor.ToFSensor.Size = [19 19]*10^-3;   
Sensor.ToFSensor.Centre = [0 0];
Sensor.ToFSensor.Shape = [-Sensor.ToFSensor.Size(1)/2 Sensor.ToFSensor.Size(2)/2;...
                          Sensor.ToFSensor.Size(1)/2 Sensor.ToFSensor.Size(2)/2;...
                          Sensor.ToFSensor.Size(1)/2 -Sensor.ToFSensor.Size(2)/2;...
                          -Sensor.ToFSensor.Size(1)/2 -Sensor.ToFSensor.Size(2)/2]';