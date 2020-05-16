function [lidarData,map2] = flyAirship(diffDrive,ppControl,initPose,goal,map1,map2,fig1,fig2,frontSensor,leftSensor,rightSensor,leftSensorOffset,rightSensorOffset,frameSize)
    sampleTime = 0.05;             % Sample time [s]
    t = 0:sampleTime:500;         % Time array
    poses = zeros(3,numel(t));    % Pose matrix
    poses(:,1) = initPose';

    % Set iteration rate
    r = rateControl(1/sampleTime);

    % Get the axes from the figures
    ax1 = fig1; %fig1.CurrentAxes;
    ax2 = fig2; %fig2.CurrentAxes;
    
    lidarData = cell(3,numel(t));
    
    for idx = 1:numel(t)
        position = poses(:,idx)';
        currPose = position(1:2);
        leftPosition = position+[0,0,leftSensorOffset];
        rightPosition = position+[0,0,rightSensorOffset];
        
        % End if pathfollowing is vehicle has reached goal position within tolerance of 0.2m
        dist = norm(goal'-currPose);
        if (dist < .2)
            disp("Goal position reached")
            break;
        end
        
        % Update map by taking sensor measurements
%         figure(fig2)
%         fig2;
        [ranges, angles] = frontSensor(position, map1);
        frontScan = lidarScan(ranges,angles);
        [ranges, angles] = leftSensor(leftPosition, map1);
        leftScan = lidarScan(ranges,angles);
        [ranges, angles] = rightSensor(rightPosition, map1);
        rightScan = lidarScan(ranges,angles);
        
        lidarData{1,idx} = frontScan;
        lidarData{2,idx} = leftScan;
        lidarData{3,idx} = rightScan;
        
        frontValidScan = removeInvalidData(frontScan,'RangeLimits',[0,frontSensor.Range(2)]);
        leftValidScan = removeInvalidData(leftScan,'RangeLimits',[0,leftSensor.Range(2)]);
        rightValidScan = removeInvalidData(rightScan,'RangeLimits',[0,rightSensor.Range(2)]);

        insertRay(map2,position,frontValidScan,frontSensor.Range(2));
        insertRay(map2,leftPosition,leftValidScan,leftSensor.Range(2));
        insertRay(map2,rightPosition,rightValidScan,rightSensor.Range(2));

        show(map2);
        
        % Run the Pure Pursuit controller and convert output to wheel speeds
        [vRef,wRef] = ppControl(poses(:,idx));
    
        % Perform forward discrete integration step
        vel = derivative(diffDrive, poses(:,idx), [vRef wRef]);
        poses(:,idx+1) = poses(:,idx) + vel*sampleTime; 
    
    
        % Update visualization
        plotTrvec = [poses(1:2, idx+1); 0];
        plotRot = axang2quat([0 0 1 poses(3, idx+1)]);
        
        % Delete image of the last robot to prevent displaying multiple robots
        if idx > 1
           items = get(ax1, 'Children');
           delete(items(1)); 
        end
    
        %plot robot onto known map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', frameSize, 'Parent', ax1);
        %plot robot on new map
        plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', frameSize, 'Parent', ax2);
    
        % waiting to iterate at the proper rate
        waitfor(r);
    end
end