function new_particles = resample(particles, weights)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
    new_particles = zeros([size(particles,1), size(particles,2)]);


    % TODO: complete this stub
    M = size(particles, 1);
    
    %Uso el Muestreo estocástico universal
    
    muestra = rand*(1/M); %Mi primera muestra es un número aleatorio entre 0 y 1/M
    j = 1; %Se corresponde a las nuevas partículas
    
    %Como los pesos estan normalizados, me sirven como umbrales entre 0 y 1.
    umbral = 0;
    
    %Itero en las M partículas
    for i = [1:M] %Se corresponde a las partículas viejas
        
        umbral = umbral + weights(i); %El umbral se actualiza cuando las muestras superan al umbral previo
        
        
        while(muestra < umbral)
            %Remuestreo tomando la partícula i hasta superar el umbral
            new_particles(j,:) = particles(i,:);
            
            %Avanzo la muestra 1/M y avanzo en mi array de nuevas
            %partículas
            muestra = muestra + 1/M;
            j = j + 1;
        end
    end
    
end
