function pl = setupPlanner(planningTs)
    pl.Ts = planningTs; % Sampling Rate 
    pl.dt = planningTs; % Prediction Interval
    
    pl.Q  = 10*diag([0.01;0.01;0.002]); % Penalty Position Regular
    pl.N  = 100; % Prediction Horizon
    pl.R  = 1*diag([0.1;0.1]); % Penalty Input
    pl.QE= diag([2;2;0.4]); % Penalty Position Terminal
    pl.v_max = 4.5; % Max Forward Vel
    pl.v_min = -4.5; % Max Backward Vel
    pl.psi_dot_max = 0.785398; % Max AntiClockwise Yawrate
    pl.psi_dot_min =-0.785398; % Max Clockwise Yawrate
    pl.a_max = 2.9; % Max Accel
    pl.a_min = -2.9; % Max Decel
    pl.ego_safety_diam= 0.6;
    
    pl.nx= 3; % Total number of states
    pl.nu= 2; % Total number of inputs
    pl.nxo=3; % Number of states in objective function
end

