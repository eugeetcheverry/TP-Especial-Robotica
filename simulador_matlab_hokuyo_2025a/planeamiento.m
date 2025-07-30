% NOTE that in octave/MATLAB, matrices are accessed with A(y,x),
% where y is the row and x is the column

function camino = planeamiento(map, start, goal)
    start = floor(start.*map.Resolution)
    goal = floor(goal.*map.Resolution)
    

    map = occupancyMatrix(map);
    start(1) = size(map, 1) - start(1);
    goal(1) = size(map, 1) - goal(1);
    % load the map
    %map = a.G;
    
    % visualizes the map. Note: 
    % the y-axis is inverted, so 0,0 is top-left, z
    figure(2)
    imshow(map);
    title ('Map');
    hold on;

    % retrieve height and width of the map
    [h,w] = size(map);

    % cost values for each cell, filled incrementally. Initialize with infinity
    costs = ones(h,w)*inf;

    % estimated costs to the goal.
    heuristics = zeros(h,w);

    % cells that have been visited
    closed_list = zeros(h,w);

    % these matrices implicitly store the path
    % by containing the x and y position of the previous
    % node, respectively. Following these starting at the goal 
    % until -1 is reached returns the computed path, see at the bottom
    previous_x = zeros(h,w)-1;
    previous_y = zeros(h,w)-1;

    %plot start and goal cell
    plot(start(2), start(1), 'r.');
    plot(goal(2), goal(1), 'g.');
    pause(1); %pause for visualization
    
    %start search at the start 
    parent=start;
    costs(start(1),start(2)) = 0;

    %loop until the goal is found
    while (parent(1)~=goal(1) || parent(2)~=goal(2))

      %generate mask to assign infinite costs for cells already visited
      closed_mask = closed_list;
      closed_mask( closed_mask==1 )=Inf; 

      %find the candidates for expansion (open list/frontier)
      open_list = costs + closed_mask + heuristics;

      %check if a non-infinite entry exists in open list (list is not empty)
      if min(open_list(:))==Inf
        disp('no valid path found');
        break
      end

      %find the cell with the minimum cost in the open list
      [y,x] = find(open_list == min(open_list(:)));
      parent_y = y(1);
      parent_x = x(1);
      parent = [parent_y, parent_x];

      %put parent in closed list
      closed_list(parent_y,parent_x) = 1;
      plot(parent_x, parent_y, ' y.' );

      %for visualization: Plot start again
      if(parent(1) == start(1) && parent(2) == start(2))
        plot(start(2), start(1), 'r.');
      end

      %get neighbors of parent
      n = neighbors(parent, [h,w]);
      for i=1:size(n,1)
        child_y = n(i,1);
        child_x = n(i,2);
        child = [child_y, child_x];

        %calculate the cost of reaching the cell
        cost_val = costs(parent_y,parent_x) + edge_cost(parent, child, map);

        %Exercise 2: estimate the remaining costs from the cell to the goal
        heuristic_val = heuristic(child, goal);

        %update cost of cell
        if cost_val < costs(child_y,child_x)
          costs(child_y,child_x) = cost_val;
          heuristics(child_y,child_x) = heuristic_val;

          %safe child's parent
          previous_x(child_y,child_x) = parent_x;
          previous_y(child_y,child_x) = parent_y;
        end
      end
      pause(0.05); %pause for visualization
    end

    % visualization: from the goal to the start,
    % draw the path as blue dots
    parent = [goal(1), goal(2)];
    camino = parent;
    distance2 = 0;
    while previous_x(parent(1), parent(2))>=0
      plot(parent(2), parent(1), 'b.');

      %for visualization: Plot goal again
      if(parent(1) == goal(1) && parent(2) == goal(2))
        plot(goal(2), goal(1), 'g.');
      end

      child_y = previous_y(parent(1), parent(2));
      child_x = previous_x(parent(1), parent(2));
      child = [child_y, child_x];
      distance2 = distance2+norm(parent - child);
      parent = child;
      pause(0.05); %pause for visualization
      camino = [camino;parent];
    end
    
    camino(:,1) = size(map, 1) - camino(:,1);
    camino = camino./25;
    camino = flip(camino,2);
    camino = flip(camino,1);
    

    disp 'done'
    disp 'path cost: ', disp(costs(goal(1),goal(2)));
    disp 'path length: ', disp(distance2);
    disp 'number of nodes visited: ', disp(sum(closed_list(:)));

end
