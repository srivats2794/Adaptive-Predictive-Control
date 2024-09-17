clc;close all;

sim_choice=1; % TURN ON IF YOU WANT TO RUN A SIM

if sim_choice
    clc;clear all;close all;
    
    perc=5; % MANIPULATE THE PARAMETER PERTURBATION PERCENTAGE
    attack=1; % TURNS ON PARAMETER PERTURBATION
    loud = 0; % Plays video of the whole simulation if its set to 1
    
    % Baseline parameters
    master.m_p_0=4;  master.l_0= 0.53;

    % Simulation time and step
    master.tsim=20; master.simTs=0.01;

    % Sine properties for l_p perturbation
    master.sine_min_freq= 0.1; master.sine_max_freq= 20; master.sine_modulation=1e-6;

    % Function that varies m_p. Ouputs m_p(t)
    master.m_vec= varyParamsDiscrete(master.m_p_0,75,randi([1 3]),randi([9 20]),master.tsim,master.simTs,1);

    % Function that varies l_p. Outputs l_p(t)
    [master.t,master.l_vec]= varyParamsSmoothSine(master.l_0,...
        perc,master.sine_min_freq,master.sine_max_freq,...
        master.sine_modulation,master.tsim,master.simTs,1,0);
    
    % Simulates the vanilla MPC... Open-> MPC/main.m for details
    cd MPC\
     loud=0; 

     main;
     clearvars -except cascaded loud attack_perc attack master
     clc;close all;
    cd ..
    
    % Simulates the Adaptive Predictive Control... APC/main.m for details
    cd APC\
     loud=0;
     main;
     clearvars -except cascaded cascaded_adaptive attack_perc master
     clc;close all;
    cd ..
end


%% PLOTTING

line_width=3;

if length(cascaded.t) > length(cascaded_adaptive.t)
    violation_t= cascaded.t;
    violation_line= cascaded.violation_line*ones(length(cascaded.t),1);
else
    violation_t= cascaded_adaptive.t;
    violation_line= cascaded_adaptive.violation_line*ones(length(cascaded_adaptive.t),1);
end

%%
figure(500)
nexttile
min_len1=min(length(cascaded.t), length(cascaded.obs_mapping(1,:)));
min_len2=min(length(cascaded_adaptive.t), length(cascaded_adaptive.obs_mapping(1,:)));
plot(cascaded.t(1:min_len1),cascaded.obs_mapping(1,1:min_len1),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.obs_mapping(1,1:min_len2),'--r',"LineWidth",line_width);
hold on
plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
hold off
ylim([0.5 max(max(cascaded.obs_mapping(1,:),max(cascaded_adaptive.obs_mapping(1,:))))]);
set(gca,"FontSize",14)
xlabel("Time(s)","FontSize",14);

title("Distance to Obstacle 1 (m)","FontSize",14)
%
nexttile
plot(cascaded.t(1:min_len1),cascaded.obs_mapping(2,1:min_len1),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.obs_mapping(2,1:min_len2),'--r',"LineWidth",line_width);
hold on
plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
hold off
ylim([0.5 max(max(cascaded.obs_mapping(2,:),max(cascaded_adaptive.obs_mapping(2,:))))]);
set(gca,"FontSize",14)
xlabel("Time(s)","FontSize",14);

title("Distance to Obstacle 2 (m)","FontSize",14)
%
nexttile
plot(cascaded.t(1:min_len1),cascaded.obs_mapping(3,1:min_len1),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.obs_mapping(3,1:min_len2),'--r',"LineWidth",line_width);
hold on
plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
hold off
ylim([0.5 max(max(cascaded.obs_mapping(3,:),max(cascaded_adaptive.obs_mapping(3,:))))]);
set(gca,"FontSize",14)
xlabel("Time(s)","FontSize",14);

title("Distance to Obstacle 3 (m)","FontSize",14)
%
nexttile
plot(cascaded.t(1:min_len1),cascaded.obs_mapping(4,1:min_len1),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.obs_mapping(4,1:min_len2),'--r',"LineWidth",line_width);
hold on
plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
hold off
ylim([0.5 max(max(cascaded.obs_mapping(4,:),max(cascaded_adaptive.obs_mapping(4,:))))]);
set(gca,"FontSize",14)
xlabel("Time(s)","FontSize",14);
legend('MPC','APC','Violation Line',"FontSize",14);
title("Distance to Obstacle 4 (m)","FontSize",14)

%%
min_len1=min(length(cascaded.t), length(cascaded.error_ego));
min_len2=min(length(cascaded_adaptive.t), length(cascaded_adaptive.error_ego));
figure(502)
plot(cascaded.t(1:min_len1),cascaded.error_ego(1:min_len1),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.error_ego(1:min_len2),'--r',"LineWidth",line_width);
set(gca,"FontSize",14)
ylabel("Norm of Pose Error","FontSize",14); xlabel("Time (s)","FontSize",14);
ylim([0 max(max(cascaded.error_ego),max(cascaded_adaptive.error_ego))+0.1]);
hold off
legend('MPC','APC',"FontSize",14);

%%
min_len1=min(length(cascaded.t), length(cascaded.control_effort));
min_len2=min(length(cascaded_adaptive.t), length(cascaded_adaptive.control_effort));
figure(503)
plot(cascaded.t(1:min_len1),cascaded.control_effort(1:min_len1),"LineWidth",line_width);
hold on
set(gca,"FontSize",14)
plot(cascaded_adaptive.t(1:min_len2),cascaded_adaptive.control_effort(1:min_len2),"LineWidth",line_width);
hold off
legend('MPC','APC',"FontSize",14);
title("Control Effort","FontSize",14)

%%

% figure(504)
% plot(cascaded.t,cascaded.exec_time,'-k',"LineWidth",line_width);
% hold on
% plot(violation_t(1:50:end),0.01*ones(length(violation_t(1:50:end)),1),'*',"MarkerSize",3*line_width,"LineWidth",0.5*line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.exec_time,'--r',"LineWidth",line_width);
% hold on
% plot(violation_t(1:50:end),0.1*ones(length(violation_t(1:50:end)),1),'+',"MarkerSize",3*line_width,"LineWidth",0.5*line_width);
% hold off
% ylim([-0.01 0.11]);
% legend('Cascaded','Cascaded Execution Deadline','APC','MRAC Execution Deadline');
% title("Execution Time (s)")
min_len1=min(length(cascaded.t), length(cascaded.exec_time));
min_len2=min(length(cascaded_adaptive.t), length(cascaded_adaptive.exec_time));
figure(504)
plot(cascaded.t(1:min_len1), cascaded.exec_time(1:min_len1), '-k', "LineWidth", line_width);
hold on
plot(violation_t(1:50:end), 0.01 * ones(length(violation_t(1:50:end)), 1), '*', "MarkerSize", 3 * line_width, "LineWidth", 0.5 * line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2), cascaded_adaptive.exec_time(1:min_len2), '--r', "LineWidth", line_width);
hold off
set(gca,"FontSize",14)
ylim([-0.002 0.011]);
xlabel("Time (s)","FontSize",14); ylabel("Execution Time (s)","FontSize",14)
legend('MPC', 'Execution Deadline', 'APC',"FontSize",14);
title("Execution Time (s)","FontSize",14)

% Find max values and their indices for annotations
[max_cascaded_exec_time, idx_cascaded] = max(cascaded.exec_time);
[max_cascaded_adaptive_exec_time, idx_cascaded_adaptive] = max(cascaded_adaptive.exec_time);

% Define deadline times
cascaded_deadline_time = 0.01;

% Add annotations for max values
text(cascaded.t(idx_cascaded), max_cascaded_exec_time, sprintf('Max MPC Exec Time: %.4f', max_cascaded_exec_time), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k',"FontSize",14);
text(cascaded_adaptive.t(idx_cascaded_adaptive), max_cascaded_adaptive_exec_time, sprintf('Max APC Exec Time: %.4f', max_cascaded_adaptive_exec_time), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'r',"FontSize",14);

% Add annotations for deadline times
text(violation_t(end), cascaded_deadline_time, sprintf('Deadline Time: %.4f', cascaded_deadline_time), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k',"FontSize",14);

%%
min_len1=min(length(cascaded.t), length(cascaded.theta));
min_len2=min(length(cascaded_adaptive.t), length(cascaded_adaptive.theta));
figure(505)
plot(cascaded.t(1:min_len1),rad2deg(cascaded.theta(1:min_len1)),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),rad2deg(cascaded_adaptive.theta(1:min_len2)),'--r',"LineWidth",line_width);
hold on
plot(violation_t(1:50:end),rad2deg(0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
    violation_t(1:50:end),rad2deg(-0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
    "MarkerSize",3*line_width,"LineWidth",line_width);
hold off
set(gca,"FontSize",14)
xlabel("Time (s)","FontSize",14); ylabel("Pitch Angle (deg)","FontSize",14)
legend('MPC','APC','Bounds',"FontSize",14);

figure(506)
plot(cascaded.t(1:min_len1),rad2deg(cascaded.thetaDot(1:min_len1)),'-k',"LineWidth",line_width);
hold on
plot(cascaded_adaptive.t(1:min_len2),rad2deg(cascaded_adaptive.thetaDot(1:min_len2)),'--r',"LineWidth",line_width);
hold on
hold on
plot(violation_t(1:50:end),rad2deg(0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
    violation_t(1:50:end),rad2deg(-0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
    "MarkerSize",3*line_width,"LineWidth",line_width);
hold off
set(gca,"FontSize",14)
legend('MPC','APC','Bounds',"FontSize",14);
xlabel("Time (s)","FontSize",14); ylabel("Pitch Rate (deg/s)","FontSize",14)

figure(507)
plot(master.t,master.m_vec,'LineWidth',2);
hold on
plot(master.t,master.m_p_0*ones(length(master.m_vec)),'LineWidth',2);
hold off
ylim([3 7]);
xlabel('Time (s)'); ylabel('Mass (Kg)');
title("Mass");

figure(508)
plot(master.t,master.l_vec,'LineWidth',2);
hold on
plot(master.t,master.l_0*ones(length(master.m_vec)),'LineWidth',2);
hold off
xlabel('Time (s)'); ylabel('Length (m)');
title("Length");