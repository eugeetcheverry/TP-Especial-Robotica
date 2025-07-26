function particles = initialize_particles(count)
    % Returns a set of randomly initialized particles.
    particles = [
        normrnd(15.0, 2, 1, 1), ...
        normrnd(15.0, 2, 1, 1), ...
        unifrnd(-pi, pi, 1, 1)
    ];
    i = 1;
    while i < count
        print('Hola estoy en el while')
        if map_is_free(particles(i, 1), particles(i, 2), [0, 0])
            i = i + 1
            aux = [
                normrnd(15.0, 2, 1, 1), ...
                normrnd(15.0, 2, 1, 1), ...
                unifrnd(-pi, pi, 1, 1)
            ];
            particles = [particles; aux];
        else 
            particles(i, :) = [
                normrnd(15.0, 2, 1, 1), ...
                normrnd(15.0, 2, 1, 1), ...
                unifrnd(-pi, pi, 1, 1)
            ];
        end
    end
end

