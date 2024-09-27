function [t, l_p_series, freqs] = varyParamsFourierSeries(l_p_0, amplitude_percentage, min_freq, max_freq, num_freqs, simulation_time, dt, change_interval, quiet)
    % Determine the number of segments based on the change_interval
    num_segments = floor(simulation_time / change_interval);
    
    % Time vector
    t = 0:dt:simulation_time;
    
    % Generate frequency arrays for each segment
    freqs = cell(1, num_segments+1);  % Include one more for the leftover time
    for i = 1:num_segments
        freqs{i} = randi([min_freq, max_freq], 1, num_freqs);
    end
    % Handle the last segment (if there's remaining time)
    freqs{end} = randi([min_freq, max_freq], 1, num_freqs);
    
    % Calculate base amplitude
    amplitude = amplitude_percentage / 100 * l_p_0;
    
    % Initialize the l_p series
    l_p_series = zeros(size(t));
    
    % Loop through each segment and add corresponding sinusoids
    for i = 1:num_segments
        t_segment = t(t >= (i-1)*change_interval & t < i*change_interval);
        for k = 1:num_freqs
            l_p_series(t >= (i-1)*change_interval & t < i*change_interval) = ...
                l_p_series(t >= (i-1)*change_interval & t < i*change_interval) + amplitude * sin(2 * pi * freqs{i}(k) * t_segment);
        end
    end
    
    % Handle the last segment (remainder time)
    t_last_segment = t(t >= num_segments*change_interval);
    for k = 1:num_freqs
        l_p_series(t >= num_segments*change_interval) = ...
            l_p_series(t >= num_segments*change_interval) + amplitude * sin(2 * pi * freqs{end}(k) * t_last_segment);
    end
    
    % Normalize the l_p series by the number of frequencies
    l_p_series = l_p_series / num_freqs;
    
    % Add the base value
    l_p_series = l_p_0 + l_p_series;
    
    % Plot the result
    if ~quiet
        figure;
        plot(t, l_p_series);
        xlabel('Time (s)');
        ylabel('Parameter Value (l_p)');
        title('Flexible Fourier Series from Random Frequencies');
        grid on;
    end
end