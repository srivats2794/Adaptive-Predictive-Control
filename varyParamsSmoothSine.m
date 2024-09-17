function [t,l_p_series] = varyParamsSmoothSine(l_p_0, amplitude_percentage, freq_min, freq_max, freq_modulation, simulation_time, dt, rand_attack, quiet)
    % Time vector
    t = 0:dt:simulation_time;
    
    % Calculate amplitude based on percentage
    amplitude = amplitude_percentage / 100 * l_p_0;
    
    % Initialize the l_p series
    l_p_series = zeros(size(t));
    
    % Set the initial parameter value
    l_p_series(1) = l_p_0;
    
    % Loop through each time step
    for i = 2:length(t)
        % Random perturbation added to the modulation signal
        if rand_attack
            factor=rand(1);
        else
            factor=1;
        end
        % Calculate smoothly varying frequency with random noise
        smooth_frequency = freq_min + (freq_max-freq_min)*factor * (0.5 + 0.5 * sin(2 * pi * freq_modulation * t(i)));
        
        % Calculate the phase increment
        phase = 2 * pi * smooth_frequency * t(i);
        
        % Calculate the sinusoidal variation
        delta_l_p = amplitude * sin(phase);
        
        % Update the parameter value
        l_p_series(i) = l_p_0 + delta_l_p;
    end
    
    if nargin<9
        quiet=1;
    end
    
    if ~quiet
        % Plot the results
        figure;
        plot(t, l_p_series);
        xlabel('Time (s)');
        ylabel('Parameter Value (l_p)');
        title('Smooth Sinusoidal Variation of Parameter with Randomness');
        grid on;
    end
end

