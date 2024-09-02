function param_vec = varyParamsDiscrete(param, amplitude_percentage, entry_time,exit_time, simulation_time, dt,quiet)

% Time vector
t = 0:dt:simulation_time;

    for i=1:length(t)
        param_vec(i)=param;
        if t(i)>=entry_time
            param_vec(i)= param+(amplitude_percentage/100)*param;
        end
        
        if t(i)>=exit_time
            param_vec(i)= param;
        end
    end

    if nargin<7
        quiet=1;
    end
    if ~quiet
        plot(t,param_vec);
    end
end

