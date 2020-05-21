function [virtualWorld,virtualWorldFigure] = Setup3DSimulationEnvironment(referenceMap,bwimage,startloc)
    % Generating boundaries
    boundaries = bwboundaries(flip(bwimage),4);
    xyBoundaries = boundaries{:};
    xyBoundaries = [xyBoundaries(:,2),xyBoundaries(:,1)];
    
    figure("Name","Boundary Plots");
    subplot(1,2,1)
    show(referenceMap);
    subplot(1,2,2)
    plot(xyBoundaries(:,1),xyBoundaries(:,2));

    % ###### Closing all vrworlds ######
    out = vrwho;
    for i=1:length(out)
        while (get(out(i),'opencount')~=0)
            close(out(i));
        end
        delete(out(i));
    end

    xmax = referenceMap.GridSize(2);
    zmax = referenceMap.GridSize(1);
    
    % ###### Loading 3D vrworld for simulation ######
    virtualWorld = vrworld('3DModel/3DModelTemplate.x3d');
    open(virtualWorld);


    % ###### Airship node ######
    airshipNode = virtualWorld.Airship;
    airshipNode.translation = [startloc(1), 1, startloc(2)];
    
    % ###### Adjusting cameras ######
    % overview viewport
    overviewViewport = virtualWorld.VP_Overview;
    overviewViewportOffset = zmax*2;
    overviewViewport.orientation = [1 0 0 -0.95];
    overviewViewport.position = [xmax/2 overviewViewportOffset overviewViewportOffset];

    topViewport = virtualWorld.VP_Top;
    topViewportHeight = max(xmax,zmax)/2*3;
    topViewport.position = [xmax/2 topViewportHeight zmax/2];

    % ###### Resizing floor ######
    floorNode = virtualWorld.Floor;
    floorNode.geometry.coord.point = [  0       0   0
                                        xmax    0   0
                                        xmax    0   zmax
                                        0       0   zmax];

    % ###### Adding walls ######
    mainWallsNode = virtualWorld.MainWalls;
    mainWallsNode.geometry.crossSection = xyBoundaries;

    % ###### Showing world ######
    virtualWorldFigure = vrfigure(virtualWorld);
end