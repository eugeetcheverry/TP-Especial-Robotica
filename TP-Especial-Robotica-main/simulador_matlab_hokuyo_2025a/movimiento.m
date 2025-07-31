function [v_cmd, w_cmd] = movimiento(pose, particles)
    theta_robot = particles(3);
    
    angle = atan2(pose(2) - particles(2), pose(1) - particles(1));
    angle_diff = wrapToPi(angle - theta_robot);
    
    if abs(angle_diff) < deg2rad(5)
        %Seguir como esty Vcmd igual wcmd cero
        v_cmd = 0.2;
        w_cmd = 0;
    elseif angle_diff > 0 %Girar a la izquied=rda
        v_cmd = 0;
        w_cmd = 0.35;
    elseif angle_diff < 0 %Girar a la derecha
        v_cmd = 0;
        w_cmd = -0.35;
    end

end