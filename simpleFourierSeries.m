function [t, l_p_series, freqs] = simpleFourierSeries(l_p_0, amplitude_percentage, min_freq, max_freq, num_freqs, simulation_time, dt, quiet)
    % Generate a random array of integers between min_freq and max_freq
    freqs = randi([min_freq, max_freq], 1, num_freqs);
    
    % Time vector
    t = 0:dt:simulation_time;
    
    % Calculate base amplitude
    amplitude = amplitude_percentage / 100 * l_p_0;
    
    % Initialize the l_p series
    l_p_series = zeros(size(t));
    
    % Loop through each random frequency and add the corresponding sinusoid
    for k = 1:num_freqs
        % Add the sinusoid with the calculated amplitude and the given random frequency
        l_p_series = l_p_series + amplitude * sin(2 * pi * freqs(k) * t);
    end
    
    % Normalize the l_p series by dividing by the number of frequencies
    l_p_series = l_p_series / num_freqs;
    
    % Add the base value
    l_p_series = l_p_0 + l_p_series;
    
    % Plot the result
    if ~quiet
        figure;
        plot(t, l_p_series);
        xlabel('Time (s)');
        ylabel('Parameter Value (l_p)');
        title('Fourier Series from Random Frequencies');
        grid on;
    end
end
