function [t, l_p_series] = varyParamsFourierRandomFreq(l_p_0, amplitude_percentage, num_freqs, freq_min, freq_max, simulation_time, dt, quiet)
    % Time vector
    t = 0:dt:simulation_time;
    
    % Calculate base amplitude
    amplitude = amplitude_percentage / 100 * l_p_0;
    
    % Randomly generate frequencies within the specified range
    freqs = freq_min + (freq_max - freq_min) * rand(1, num_freqs);
    
    % Initialize the l_p series
    l_p_series = zeros(size(t));
    
    % Loop through each frequency and sum the sinusoids
    for k = 1:num_freqs
        % No randomness for amplitude or phase shift
        phase_shift = 0;  % You can set this to any fixed value or retain it as 0
        
        % Add the sinusoid with a fixed amplitude
        l_p_series = l_p_series + amplitude * sin(2 * pi * freqs(k) * t + phase_shift);
    end
    
    % Add the base value
    l_p_series = l_p_0 + l_p_series;
    
    if nargin < 7
        quiet = 1;
    end
    
    if ~quiet
        % Plot the results
        figure;
        plot(t, l_p_series);
        xlabel('Time (s)');
        ylabel('Parameter Value (l_p)');
        title('Fourier-Type Variation of Parameter with Random Frequencies');
        grid on;
    end
end


