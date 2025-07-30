function movimiento(pose, particles):
    theta_robot = cumsum(particles(3, :))/size(particles(3,:));
    for i = [0:size(pose))]
        angle = atan2(pose(2, i+1) - pose(2, i), pose(1, i+1) - pose(1, i));
        if (theta_robot - 0.02*pi) < angle < (theta_robot + 0.03*pi)
            %Seguir como esty Vcmd igual wcmd cero
            
        elseif angle > theta_robot
            %girar a la izquierda
        elseif angle < theta_robot
            %girar a la derecha
    end
end