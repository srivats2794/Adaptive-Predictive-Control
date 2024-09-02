function [t,l_p_series] = varyParamsSmoothSine(l_p_0, amplitude_percentage, freq_min, freq_max, freq_modulation, simulation_time, dt,quiet)
    % Parameters:
    % l_p_0            - Initial parameter value
    % amplitude_percentage - Amplitude as a percentage of l_p_0
    % freq_min         - Minimum frequency (Hz)
    % freq_max         - Maximum frequency (Hz)
    % freq_modulation  - Frequency of the modulation signal (Hz)
    % simulation_time  - Total simulation time (seconds)
    % dt               - Time step for simulation

    % Time vector
    t = 0:dt:simulation_time;
    
    % Calculate amplitude based on percentage
    amplitude = amplitude_percentage / 100 * l_p_0;
    
    % Initialize the l_p series
    l_p_series = zeros(size(t));
    
    % Set the initial parameter value
    l_p_series(1) = l_p_0;
    
    % Calculate modulation signal for frequency variation
    modulation_signal = 0.5 * (1 + sin(2 * pi * freq_modulation * t));
    
    % Loop through each time step
    for i = 2:length(t)
        % Calculate smoothly varying frequency
        smooth_frequency = freq_min + (freq_max - freq_min) * modulation_signal(i);
        
        % Calculate the phase increment
        phase = 2 * pi * smooth_frequency * t(i);
        
        % Calculate the sinusoidal variation
        delta_l_p = amplitude * sin(phase);
        
        % Update the parameter value
        l_p_series(i) = l_p_0 + delta_l_p;
    end
    if nargin<8
        quiet=1;
    end

    if ~quiet
        % Plot the results
        figure;
        plot(t, l_p_series);
        xlabel('Time (s)');
        ylabel('Parameter Value (l_p)');
        title('Smooth Sinusoidal Variation of Parameter');
        grid on;
    end
end
