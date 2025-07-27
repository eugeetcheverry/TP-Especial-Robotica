function particles = initialize_particles(count)
    % Returns a set of randomly initialized particles.
    particles = [
        unifrnd(0, 35, 1, 1), ...
        unifrnd(0, 35, 1, 1), ...
        unifrnd(-pi, pi, 1, 1)
    ];
    i = 1;
    while i < count
        if map_is_free(particles(i, 1), particles(i, 2), [0, 0])
            i = i + 1;
            aux = [
                unifrnd(0, 35, 1, 1), ...
                unifrnd(0, 35, 1, 1), ...
                unifrnd(-pi, pi, 1, 1)
            ];
            particles = [particles; aux];
        else 
            particles(i, :) = [
                unifrnd(0, 35, 1, 1), ...
                unifrnd(0, 35, 1, 1), ...
                unifrnd(-pi, pi, 1, 1)
            ];
        end
    end
end

