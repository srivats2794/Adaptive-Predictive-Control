clc;close all;
sim_choice=1;

if sim_choice
    clear all;
    sim_choice=1;
end

if sim_choice
    perc=10;
    for i_main=1:100
        i_main
        clearvars -except run perc i_main;close all;
        cd Cascased\
        plotter_choice=0;attack=1;

        attack_perc.j_m_th=randi([-perc perc]);
        attack_perc.j_p_th=randi([-perc perc]);
        attack_perc.m_p=randi([-perc perc]);
        attack_perc.l=randi([-perc perc]);
        attack_perc.j_psi=randi([-perc perc]);
        main;
        clearvars -except cascaded plotter_choice attack_perc attack perc i_main run
        run.casc_viol(i_main)=cascaded.obst_viol;
        run.casc_NOconv(i_main)= cascaded.no_convergence;
        run.casc_fail(i_main)=cascaded.failure;
        run.attack_perc(i_main)=attack_perc;
        close all;
        cd ..

        cd Cascaded_Adaptive\
        plotter_choice=0;
        main;
        clearvars -except cascaded cascaded_adaptive attack_perc perc i_main run
        run.cascA_viol(i_main)= cascaded_adaptive.obst_viol;
        run.cascA_fail(i_main)=cascaded_adaptive.failure;
        run.cascA_NOconv(i_main)= cascaded_adaptive.no_convergence;
        close all;
        cd ..
    end
end

line_width=2;

if length(cascaded.t) > length(cascaded_adaptive.t)
    violation_t= cascaded.t;
    violation_line= cascaded.violation_line*ones(length(cascaded.t),1);
else
    violation_t= cascaded_adaptive.t;
    violation_line= cascaded_adaptive.violation_line*ones(length(cascaded_adaptive.t),1);
end

%%
% 
% figure(500)
% nexttile
% plot(cascaded.t,cascaded.obs_mapping(1,:),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.obs_mapping(1,:),'--r',"LineWidth",line_width);
% hold on
% plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
% hold off
% ylim([0.5 max(max(cascaded.obs_mapping(1,:),max(cascaded_adaptive.obs_mapping(1,:))))]);
% xlabel("Time(s)");
% legend('MPC','APC','Violation Line');
% title("Distance to Obstacle 1 (m)")
% %
% nexttile
% plot(cascaded.t,cascaded.obs_mapping(2,:),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.obs_mapping(2,:),'--r',"LineWidth",line_width);
% hold on
% plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
% hold off
% ylim([0.5 max(max(cascaded.obs_mapping(2,:),max(cascaded_adaptive.obs_mapping(2,:))))]);
% xlabel("Time(s)");
% legend('MPC','APC','Violation Line');
% title("Distance to Obstacle 2 (m)")
% %
% nexttile
% plot(cascaded.t,cascaded.obs_mapping(3,:),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.obs_mapping(3,:),'--r',"LineWidth",line_width);
% hold on
% plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
% hold off
% ylim([0.5 max(max(cascaded.obs_mapping(3,:),max(cascaded_adaptive.obs_mapping(3,:))))]);
% xlabel("Time(s)");
% legend('MPC','APC','Violation Line');
% title("Distance to Obstacle 3 (m)")
% %
% nexttile
% plot(cascaded.t,cascaded.obs_mapping(4,:),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.obs_mapping(4,:),'--r',"LineWidth",line_width);
% hold on
% plot(violation_t,violation_line,'-.b',"LineWidth",line_width);
% hold off
% ylim([0.5 max(max(cascaded.obs_mapping(4,:),max(cascaded_adaptive.obs_mapping(4,:))))]);
% xlabel("Time(s)");
% legend('MPC','APC','Violation Line');
% title("Distance to Obstacle 4 (m)")
% 
% %%
% 
% figure(502)
% plot(cascaded.t,cascaded.error_ego(1,1:end),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.error_ego(1,1:end),'--r',"LineWidth",line_width);
% ylabel("Norm of Pose Error"); xlabel("Time (s)");
% ylim([0 max(max(cascaded.error_ego),max(cascaded_adaptive.error_ego))+0.1]);
% hold off
% legend('MPC','APC','FontSize',12);
% 
% %%
% 
% figure(503)
% plot(cascaded.t,cascaded.control_effort,"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,cascaded_adaptive.control_effort,"LineWidth",line_width);
% hold off
% legend('MPC','APC');
% title("Control Effort")
% 
% %%
% 
% % figure(504)
% % plot(cascaded.t,cascaded.exec_time,'-k',"LineWidth",line_width);
% % hold on
% % plot(violation_t(1:50:end),0.01*ones(length(violation_t(1:50:end)),1),'*',"MarkerSize",3*line_width,"LineWidth",0.5*line_width);
% % hold on
% % plot(cascaded_adaptive.t,cascaded_adaptive.exec_time,'--r',"LineWidth",line_width);
% % hold on
% % plot(violation_t(1:50:end),0.1*ones(length(violation_t(1:50:end)),1),'+',"MarkerSize",3*line_width,"LineWidth",0.5*line_width);
% % hold off
% % ylim([-0.01 0.11]);
% % legend('Cascaded','Cascaded Execution Deadline','APC','MRAC Execution Deadline');
% % title("Execution Time (s)")
% 
% figure(504)
% plot(cascaded.t, cascaded.exec_time, '-k', "LineWidth", line_width);
% hold on
% plot(violation_t(1:50:end), 0.01 * ones(length(violation_t(1:50:end)), 1), '*', "MarkerSize", 3 * line_width, "LineWidth", 0.5 * line_width);
% hold on
% plot(cascaded_adaptive.t, cascaded_adaptive.exec_time, '--r', "LineWidth", line_width);
% hold off
% ylim([-0.01 0.011]);
% xlabel("Time (s)"); ylabel("Execution Time (s)")
% legend('MPC', 'Execution Deadline', 'APC');
% title("Execution Time (s)")
% 
% % Find max values and their indices for annotations
% [max_cascaded_exec_time, idx_cascaded] = max(cascaded.exec_time);
% [max_cascaded_adaptive_exec_time, idx_cascaded_adaptive] = max(cascaded_adaptive.exec_time);
% 
% % Define deadline times
% cascaded_deadline_time = 0.01;
% 
% % Add annotations for max values
% text(cascaded.t(idx_cascaded), max_cascaded_exec_time, sprintf('Max MPC Exec Time: %.4f', max_cascaded_exec_time), ...
%     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k');
% text(cascaded_adaptive.t(idx_cascaded_adaptive), max_cascaded_adaptive_exec_time, sprintf('Max APC Exec Time: %.4f', max_cascaded_adaptive_exec_time), ...
%     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'r');
% 
% % Add annotations for deadline times
% text(violation_t(end), cascaded_deadline_time, sprintf('Deadline Time: %.4f', cascaded_deadline_time), ...
%     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'k');
% figure(505)
% nexttile
% plot(cascaded.t,rad2deg(cascaded.theta),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,rad2deg(cascaded_adaptive.theta),'--r',"LineWidth",line_width);
% hold on
% plot(violation_t(1:50:end),rad2deg(0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
%     violation_t(1:50:end),rad2deg(-0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
%     "MarkerSize",3*line_width,"LineWidth",line_width);
% hold off
% xlabel("Time (s)"); ylabel("Pitch Angle (deg)")
% legend('MPC','APC','Bounds',"FontSize",14);
% 
% nexttile
% plot(cascaded.t,rad2deg(cascaded.thetaDot),'-k',"LineWidth",line_width);
% hold on
% plot(cascaded_adaptive.t,rad2deg(cascaded_adaptive.thetaDot),'--r',"LineWidth",line_width);
% hold on
% hold on
% plot(violation_t(1:50:end),rad2deg(0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
%     violation_t(1:50:end),rad2deg(-0.1)*ones(length(violation_t(1:50:end)),1),'*b',...
%     "MarkerSize",3*line_width,"LineWidth",line_width);
% hold off
% xlabel("Time (s)"); ylabel("Pitch Rate (deg/s)")
