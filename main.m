% Define a small 5x5x5 3D grid as a logical array, 0s represent free space, 1s represent obstacles
% For simplicity, this example will just have a single obstacle column in the middle of the grid
input_map = false(5,5,5);
input_map(:,3,:) = true; % Setting the middle column as obstacles

% Clear a path through the obstacle
input_map(3,3,:) = false;

input_map
% Start and destination coordinates [row, column, z]
start_coords = [1, 1, 1];
dest_coords = [5, 5, 5];

[route, numExpanded] = AStarGrid3D(input_map, start_coords, dest_coords);

if isempty(route)
    disp('No path found.');
else
    [row, col, z] = ind2sub(size(input_map), route);
 
end

% Visualizing the map and the path in 3D
figure;
hold on;

% Plot obstacles
[obstacleX, obstacleY, obstacleZ] = ind2sub(size(input_map), find(input_map));
scatter3(obstacleY, obstacleX, obstacleZ, 'filled', 'MarkerFaceColor', 'k');

if ~isempty(route)
    % Convert route to subscript indices for visualization
    [row, col, z] = ind2sub(size(input_map), route);
    
    % Plot the route
    scatter3(col, row, z, 'filled', 'MarkerFaceColor', 'r');
end

% Plot start and end points
scatter3(start_coords(2), start_coords(1), start_coords(3), 100, 'o', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
scatter3(dest_coords(2), dest_coords(1), dest_coords(3), 100, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');

title('3D Motion Planning with A* Algorithm');
xlabel('Y-axis');
ylabel('X-axis');
zlabel('Z-axis');
grid on;
axis equal;
view(3); % Adjust the view angle for better visualization
hold off;
