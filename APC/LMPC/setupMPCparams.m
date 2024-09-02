function ctrl=setupMPCparams(v_min,v_max)
ctrl.tau_min= -11.5; % Max Reverse Torque

ctrl.tau_max= -ctrl.tau_min; % Max Forward Torque

ctrl.Q= diag(3*[0.1;0.1;5;3]);
ctrl.R= diag(2*[0.01;0.01]);
ctrl.QT= diag(5*[0.05;0.05;5;3]);

ctrl.N=100;

ctrl.x_max= [v_max;v_max;0.1;0.1];
ctrl.x_min= [v_min;v_min;-0.1;-0.1];
end