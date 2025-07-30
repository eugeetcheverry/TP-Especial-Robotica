function cmd = movimiento(pose, particles):
    theta_robot = cumsum(particles(3, :))/size(particles(3,:));
    for i = [0:size(pose))]
        angle = atan2(pose(2, i+1) - pose(2, i), pose(1, i+1) - pose(1, i));
        if (theta_robot - 0.02*pi) < angle < (theta_robot + 0.03*pi)
            %Seguir como esty Vcmd igual wcmd cero
            v_cmd = 0.5;
            w_cmd = 0;
        elseif angle > theta_robot %Girar a la izquied=rda
            v_cmd = 0;
            w_cmd = 0.35;
        elseif angle < theta_robot %Girar a la derecha
            v_cmd = 0;
            w_cmd = -0.35;
        end
        cmd(i) = [v_cmd, w_cmd];
    end 
end