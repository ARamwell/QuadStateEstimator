tileFig=gcf;

% 2. Specify which tile you want to extract (e.g., Tile 2)
tileNumber = 14; 

% 3. Find the specific axes in that tile and copy it to a new figure
targetAxes = tileFig.Children(1).Children(tileNumber);
newFig = figure('Name', sprintf('Extracted Tile %d', tileNumber));
copyobj(targetAxes, newFig);