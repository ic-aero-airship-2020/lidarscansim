function [refFig,referenceMap,manualPath,frameSize] = SetupSimulationEnvironment(complexity)
    switch complexity
        case 1
            load("exampleMaps.mat", "simpleMap");
            referenceMap = binaryOccupancyMap(simpleMap,1);
            frameSize = 1;
        case 2
            load("exampleMaps.mat", "complexMap");
            referenceMap = binaryOccupancyMap(complexMap,1);
            frameSize = 1;
        case 3
            actualLayout = imread('Map/Actual Layout.png');
            simplifiedLayout = imread('Map/Simplified 2D Layout.png');
            bwImage = simplifiedLayout > 0;
            referenceMap = binaryOccupancyMap(bwImage,8);
            
            figure('Name','Comparing layout');
            subplot(2,2,1);
            imshow(actualLayout);
            title("Actual Layout");
            subplot(2,2,2);
            imshow(imcomplement(bwImage));
            title("Simplified Layout");
            subplot(2,2,3);
            show(referenceMap)
            frameSize = 3;
        otherwise
            load("exampleMaps.mat", "simpleMap");
            referenceMap = binaryOccupancyMap(simpleMap,1);
            frameSize = 1;
    end   
    
    refFig = figure('Name','Reference Map','WindowState', 'maximized');
    subplot(1,2,1)
    show(referenceMap)
    
    switch complexity
       case 1
            manualPath = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
        case 2
            manualPath = [4.5 3.5; 7.5 9.5; 16.5 9.5; 15.5 15.5; 2.5 15.5; 2.5 28.5; 46.5 28.5; 48.5 14.5; 40.5 18.5; 31.5 13.5; 32.5 9.5; 46.5 8.5; 46.5 4.5];
        case 3
            manualPath = [186.4, 108.2;
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
        otherwise
            manualPath = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
    end
    % Manual driving path plotted
    figure(refFig)
    subplot(1,2,1)
    hold on
    plot(manualPath(:,1),manualPath(:,2), 'o-');
    legend("Manual driving path")
    title("Actual layout")
    hold off
end