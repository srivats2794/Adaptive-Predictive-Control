act_safety_diam=(pl.ego_safety_diam);
figure(500)
nexttile
plot(t,obs_mapping(1,:),t,...
    (act_safety_diam/2 + sim.obs_diam/2)*ones(length(t),1),...
    "LineWidth",2);
legend('Ego Trace','Violation Line');
title("Distance to Obstacle 1")
nexttile
plot(t,obs_mapping(2,:),t,...
    (act_safety_diam/2 + sim.obs_diam/2)*ones(length(t),1),...
    "LineWidth",2);
legend('Ego Trace','Violation Line');
title("Distance to Obstacle 2")
nexttile
plot(t,obs_mapping(3,:),t,...
    (act_safety_diam/2 + sim.obs_diam/2)*ones(length(t),1),...
    "LineWidth",2);
legend('Ego Trace','Violation Line');
title("Distance to Obstacle 3")
nexttile
plot(t,obs_mapping(4,:),t,...
    (act_safety_diam/2 + sim.obs_diam/2)*ones(length(t),1),...
    "LineWidth",2);
legend('Ego Trace','Violation Line');
title("Distance to Obstacle 4")

figure(502)
nexttile
plot(t,error_ego,...
    "LineWidth",2);
title("Error to Reference")
nexttile
plot(t,control_effort,...
    "LineWidth",2);
title("Control Effort")
nexttile
plot(t,per_loop_time_mod,...
    "LineWidth",2);
title("Execution Time")