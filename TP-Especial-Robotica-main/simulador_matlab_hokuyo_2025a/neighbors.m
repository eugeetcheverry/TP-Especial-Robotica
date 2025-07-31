function n = neighbors(cell, map_dimensions)

  n = [];

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
  %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
  
  %Vector con todos los posibles vecinos
  n_priori = [pos_y+1, pos_x-1; 
      pos_y+1, pos_x; 
      pos_y+1, pos_x+1; 
      pos_y, pos_x-1; 
      pos_y, pos_x+1; 
      pos_y-1, pos_x-1;
      pos_y-1, pos_x;
      pos_y-1, pos_x+1]
  
  j=1
  
  %Filtrado de los vecinos que estan fuera del mapa
  for i = 1:8
      if 0 < n_priori(i,1) && n_priori(i,1) < size_y && n_priori(i,2) < size_x && 0 < n_priori(i,2)  
        n(j,1) = n_priori(i,1);
        n(j,2) = n_priori(i,2);
      end
      j = j+1;
  end
  
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]

end
