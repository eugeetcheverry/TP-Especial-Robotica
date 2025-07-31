function is_free = map_is_free(x, y, origin)
    % x, y: position of particle in meters
    % map: occupancy grid (2D matrix)
    % res: resolution [m/cell]
    % origin: [x0, y0] world coordinates of map(1,1)
    load('mapa_TP_2025a.mat');
    % Convert world position to grid indices
    j = round((x - origin(1))* map.Resolution) + 1;
    i = round((y - origin(2))* map.Resolution) + 1;

    % Default: invalid (e.g., out of bounds)
    is_free = false;
    % Check map bounds
    if i < 1 || i > map.GridSize(1)-1 || j < 1 || j > map.GridSize(2)-1
        return;
    end

    % Get cell value
    cell_val = getOccupancy(map, [x, y]);

    % You can adjust this logic depending on how your map is defined
    % For example: 0 = free, 1 = occupied
    if cell_val < 0.1
        is_free = true;
    end
end
