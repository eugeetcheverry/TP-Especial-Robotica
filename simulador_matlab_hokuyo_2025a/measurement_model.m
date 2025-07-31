function weight = measurement_model(z, x, lidar)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of landmark observations. Each observation contains the id of the landmark observed in z(i).id and the measured range in z(i).range.
    % x: set of current particles
    % l: map of the environment composed of all landmarks
    sigma = 1;
    weight = ones(size(x, 1), 1);
    
    z_esp = zeros(floor(size(z,1)/9),size(x,1));
    
    for i = 1:size(x,1)
        z_esp(:, i) = lidar(x(i,:)');
    end

    z_trunc = z(1:9:size(z,1)-1);
    
    if size(z, 1) == 0
        return
    end
    
    z_esp(isnan(z_esp)) = 5.0;
    z_trunc(isnan(z_trunc)) = 5.0;
    
    z_mat = repmat(z_trunc(:), 1, size(x,1));
    
    coef = 1 / (sqrt(2*pi)*sigma);
    exponent = -0.5 * ((z_mat - z_esp) ./ sigma).^2;
    prob = coef * exp(exponent);

    % Multiplicar probabilidades por partícula (cada columna)
    weight = prod(prob, 1)';  % Resultado: vector columna [N x 1]
    
      
    
    %{
    for i = 1:size(z_esp, 1)
        range = z_trunc(i);
        if isnan(range)
            range = 5.0;
        end

        %TODO: compute weight
        for j = 1:size(x,1)
            range_esp = z_esp(i, j);
            if isnan(range_esp)
                range_esp = 5.0;
            end
            %En cada iteración de j calculo la distancia entre la partícula y el
            %marcador
            
            %El peso para cada partícula se actualiza al multiplicarlo por
            %la proba de que la medición sobre ese marcador se corresponda con la distancia
            %entre la partícula y el marcador
            weight(j) = weight(j)*normpdf(range, range_esp, sigma); %Aplico ruido gaussiano
        end

    end
    %}
    weight = weight ./ (size(z, 2)/9);
end

