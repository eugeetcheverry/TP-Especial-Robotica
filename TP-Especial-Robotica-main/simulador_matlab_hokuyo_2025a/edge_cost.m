function cost = edge_cost(parent, child, map)

  cost = 0;
 
  %%% YOUR CODE FOR CALCULATING THE COST FROM VERTEX parent TO VERTEX child GOES HERE
  if map(child(1),child(2)) >= 0.3
      cost = inf; %Costo inf para celdas probablemente ocupadas
  else
      cost = norm(parent - child)*(1+2*map(child(1),child(2))) 
      %El 1+map para que elija las celdas con menor ocupación pero sin llegar a que el camino 
      %optimo tenga menor costo que la heurísitca de A*;
  end
end
