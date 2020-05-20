function [plannedPoses,planFig] = PlanPath(generatedMap,start,goal,minTurningRadius,maxAirshipHalfWidth,validationDistance)
    planFig = figure("Name","Planned path",'WindowState', 'maximized');
    subplot(1,2,1)
    show(generatedMap)
    
    % Show the start and goal positions of the robot
    hold on
    plot(start(1), start(2), 'ro')
    plot(goal(1), goal(2), 'mo')
    
    % Show the start and goal headings
    r = 0.5;
    plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-' )
    plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-' )
    hold off
    
    % Defining state space of the vehicle
    bounds = [generatedMap.XWorldLimits; generatedMap.YWorldLimits; [-pi pi]];
    
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = minTurningRadius;
    
    % map is inflated to account for the width of the airship
    stateValidator = validatorOccupancyMap(ss); 
    mapInflated = copy(generatedMap);
    inflate(mapInflated, maxAirshipHalfWidth);
    stateValidator.Map = mapInflated;
    
    % Defining validation distance
    stateValidator.ValidationDistance = validationDistance; % m 
    
    % ########## Edit this ###########
    % Choosing the type of planner and defining the parameters
    % Using rapidly exploring random tree planning algorithmn
    planner = plannerRRT(ss, stateValidator);
    planner.MaxConnectionDistance = 2.0;
    planner.MaxIterations = 30000;

    planner.GoalReachedFcn = @CheckIfGoalReached;
    % ################################
    
    rng(0,'twister')
    [pthObj, solnInfo] = plan(planner, start, goal);
    
    % Plotting the path
%     planFig = figure("Name","Planned path")
    subplot(1,2,2)
    show(generatedMap)
    hold on

    % Search tree
    plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

    % Interpolate and plot path
    interpolate(pthObj,300)
    plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

    % Show the start and goal in the grid map
    plot(start(1), start(2), 'ro')
    plot(goal(1), goal(2), 'mo')
    hold off
    
    plannedPoses = pthObj.States(:,1:2);
end