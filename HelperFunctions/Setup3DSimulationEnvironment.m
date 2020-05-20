% function Setup3DSimulationEnvironment(referenceMap)
%     
%     
% end

% Closing all vrworlds
out = vrwho;
for i=1:length(out)
    while (get(out(i),'opencount')~=0)
        close(out(i));
    end
    delete(out(i));
end

% Creating new vrworld for simulation
virtualWorld = vrworld('');
open(virtualWorld);

% Creating landscape
shapeName = ['Landscape'];
newShape = vrnode(myworld,shapeName,'Shape');
newGrid = vrnode(newShape,'geometry',...
    'DEM','ElevationGrid');

