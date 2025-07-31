function heur = heuristic(cell, goal)
  
  heur = 0;
  
  %%% YOUR CODE FOR CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL GOES HERE
  
  heur = 3*norm(cell-goal); %Distancia euclídea como heurística, se multiplica por un factor para el último ej
  
end
