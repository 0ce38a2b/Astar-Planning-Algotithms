function [route, numExpanded] = AStarGrid3D(input_map, start_coords, dest_coords)
% Run A* algorithm on a 3D grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1 in a 3D space
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first two entries are the row and the column, the third is the depth.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: The total number of nodes expanded during the search.

[nrows, ncols, ndepths] = size(input_map);

% Initialize map to track state of each grid cell in 3D
map = zeros(nrows, ncols, ndepths);
map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes in 3D
start_node = sub2ind(size(map), start_coords(1), start_coords(2), start_coords(3));
dest_node  = sub2ind(size(map), dest_coords(1), dest_coords(2), dest_coords(3));

% Set up parent matrix to keep track of the path
parent = zeros(nrows, ncols, ndepths);

% Heuristic function, H, for each grid cell using Manhattan distance in 3D
[X, Y, Z] = meshgrid(1:ncols, 1:nrows, 1:ndepths);
H = abs(X - dest_coords(2)) + abs(Y - dest_coords(1)) + abs(Z - dest_coords(3));
H = permute(H, [2, 1, 3]); % Adjust dimensions to match input_map

% Initialize cost arrays
f = Inf(nrows, ncols, ndepths);
g = Inf(nrows, ncols, ndepths);

g(start_node) = 0;
f(start_node) = H(start_node);

% Initialize the number of nodes that are expanded
numExpanded = 0;

% Main Loop
while true
    % Find the node with the minimum f value
    [min_f, current] = min(f(:));
    
    if (current == dest_node) || isinf(min_f)
        break;
    end;
    
    % Mark current node as visited
    f(current) = Inf;
    numExpanded = numExpanded + 1;
    
    % Compute 3D coordinates of current node
    [i, j, k] = ind2sub(size(f), current);
    
    % Visit all of the neighbors around the current node and update the
    % entries in the f, g, and parent arrays
    neighbors = AStarNeighbors3D(map, i, j, k);
    
    for index = 1:length(neighbors)
        neighbor = neighbors(index);
        [ni, nj, nk] = ind2sub(size(map), neighbor);
        
        if g(neighbor) > g(current) + 1
            g(neighbor) = g(current) + 1;
            f(neighbor) = g(neighbor) + H(neighbor);
            parent(neighbor) = current;
        end
    end
end

% Construct route from start to dest by following the parent links
if isinf(f(dest_node))
    route = [];
    disp('No path found.');
else
    route = dest_node;
    
    while parent(route(1)) ~= 0
        route = [parent(route(1)), route];
    end
    
    disp(['Number of expanded nodes: ', numExpanded]);
    disp('Path found.');
end

end

function neighbors = AStarNeighbors3D(map, i, j, k)
% Find the neighbors of cell (i, j, k) on a 3D grid
[nrows, ncols, ndepths] = size(map);
neighbors = [];

directions = [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1];
for d = 1:size(directions, 1)
    ni = i + directions(d, 1);
    nj = j + directions(d, 2);
    nk = k + directions(d, 3);
    if ni > 0 && ni <= nrows && nj > 0 && nj <= ncols && nk > 0 && nk <= ndepths
        if map(ni, nj, nk) ~= 2 % If not an obstacle
            neighbors(end+1) = sub2ind(size(map), ni, nj, nk);
        end
    end
end

end


