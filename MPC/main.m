% clc;clear all;close all;
% 
% loud=1; % 0 or 1

%% SETUP ENV
scene = 1; % 1 or 2
addpath("PLANT\","LMPC\","HELPERS\","PLANNER\");

%% SAMPLING TIMES
simTs=master.simTs; % Simulation timestep
ctrlTs=0.01; % Control timestep
planningTs=0.1; % Planning timestep

%% Simulation Parameters
robot= ROBOT(setupRobotParams,simTs,1); % Plant

sim.x0= [0.75 ; 0.65 ;0; 0; 0;0;0]; % Initial condition
sim.xf= [1.8; 3.25 ; pi/2; 0; 0;0;0]; % Final condition
sim.obs_num=4; sim.obs_diam = 0.6; % Obstacle Info
if scene==1
    sim.obs_x= [0.9;2.2;1.1;2.5;]; % Obstacle X positions
    sim.obs_y= [1.5;1.75;2.8;3]; % Obstacle Y positions
else
    sim.obs_x= [0.3;0.9;1.5;2.1;]; % Obstacle X positions
    sim.obs_y= [1.5;1.5;1.5;1.5];% Obstacle Y positions
end

sim.tsim=  master.tsim; % Max Time
sim.x_min= 0; sim.x_max= 3; sim.y_min= 0; sim.y_max= 5; % Map bounds

%% Planner Initialization - NMPC Planner
% CasADi solver setup
pl=setupPlanner(planningTs);
[pl_solver,pl_args,f_temp]= pl_prob_setup(pl,sim,robot);

%% Controller Initialization
ctrl=LMPC(ctrlTs,1,setupMPCparams(pl.v_min,pl.v_max),setupMPCsystem,loud);

%% Viz Setup - For simulation graphics visualization
viz.w= robot.w;
viz.l= robot.r_w*2+0.02;
viz.marker_h= viz.l;
viz.marker_w= viz.w/4;
viz.map_x= [0, 3, 3, 0, 0];
viz.map_y= [0, 0, 4.5, 4.5, 0];
viz.ang= 0:0.005:2*pi;
viz.obs_circ_x= sim.obs_diam/2*cos(viz.ang);
viz.obs_circ_y= sim.obs_diam/2*sin(viz.ang);
viz.rob_circ_x= (pl.ego_safety_diam/2)*cos(viz.ang);
viz.rob_circ_y= (pl.ego_safety_diam/2)*sin(viz.ang);

%% Run Simulation
simulation;
rmpath("PLANT\","LMPC\","HELPERS\","PLANNER\");
%% Run Visualization
if loud
    sim_viz(pl,sim,viz,pos_fbk_vec,pl_rec,'Vanilla MPC');
end

cascaded.t= t;
cascaded.obs_mapping= obs_mapping;
cascaded.violation_line= pl.ego_safety_diam/2 + sim.obs_diam/2;
cascaded.error_ego= error_ego;
cascaded.control_effort= control_effort;
cascaded.exec_time= per_loop_time_mod;
cascaded.theta= fbk_vec(6,1:end-1);
cascaded.thetaDot= fbk_vec(7,1:end-1);
cascaded.failure=failure;
cascaded.obst_viol= ~isempty(cascaded.obs_mapping(cascaded.obs_mapping<cascaded.violation_line));
cascaded.no_convergence= (isempty(cascaded.error_ego(cascaded.error_ego<=0.009)));