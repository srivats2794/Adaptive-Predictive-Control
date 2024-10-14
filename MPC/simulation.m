%% SETUP

fbk=sim.x0;
fbk_vec(:,1) = fbk;

% two inputs for robot at planner level
u0_pl = zeros(pl.N,2);       
% initialization of the kinematic states decision variables
X0_pl = repmat(sim.x0(1:pl.nx),1,pl.N+1)'; 

u_ref=[0;0];

pl_rec = [];
ctrl_ref=[];

K= floor(sim.tsim/ctrl.Ts);

pl_exec_freq= round(pl.Ts/ctrl.Ts);
% Create a relative time vector for planning reference
t_pl_vel= (linspace(0,(pl.N*pl.dt)-pl.dt,pl.N))';
t_pl_pos= (linspace(0,((pl.N+1)*pl.dt)-pl.dt,pl.N+1))';
i=1;

tau_l_prev=0;
tau_r_prev=0;
pl_u= 0.01*(ones(pl.N,2));
pl_loop = tic;

%% MAIN LOOP
for j=1:K
       
    if(norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2) < 0.009)
        % If error has converged, end simulation
        error_ego(j)= norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2);
        break;
    end
    
    for iter=1:sim.obs_num
        % Record distance to obstacle for metric
     obs_mapping(iter,j) =norm(fbk(1:2)-[sim.obs_x(iter);sim.obs_y(iter)],2);
    end

    % Record distance to final position for metric
    error_ego(j)= norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2);

    t(j) = (j-1)*ctrl.Ts;
   
    fbk_lin= [fbk(4:7)];
    
    if(rem(j,pl_exec_freq)==1) %% Planning Cycle

        % Replace initial index of warm start with feedback
        X0_pl(1,:) = fbk(1:pl.nx)';

        % Compute terminal cost for planning NMPC
        QT= computeTerminalCost(sim.xf(1:pl.nxo),pl_u(end,1),pl_u(end,2),pl.Q,pl.R);

        % Live parameters of planning optimization
        pl_args.p   = [fbk(1:pl.nx);sim.xf(1:pl.nxo);u_ref;QT(:)];
        
        % Initial conditions for planning optimization 
        pl_args.x0  = [reshape(X0_pl',pl.nx*(pl.N+1),1);reshape(u0_pl',pl.nu*pl.N,1)];

        % Solve planning NMPC
        pl_sol = pl_solver('x0', pl_args.x0, 'lbx', pl_args.lbx, 'ubx', pl_args.ubx,...
            'lbg', pl_args.lbg, 'ubg', pl_args.ubg,'p',pl_args.p);

        % get inputs vector from the planning solution
        pl_u = reshape(full(pl_sol.x(pl.nx*(pl.N+1)+1:end))',pl.nu,pl.N)';
        % get state vector of solution TRAJECTORY for plotting purposes
        pl_st= reshape(full(pl_sol.x(1:pl.nx*(pl.N+1)))',pl.nx,pl.N+1)';
        pl_rec(:,1:pl.nx,i)= pl_st(:,1:3);
        X0_pl = reshape(full(pl_sol.x(1:pl.nx*(pl.N+1)))',pl.nx,pl.N+1)'; 
        
        % Planning's input vector is control's reference vector
        ctrl_ref= pl_u';

        % record fbk for plotting purposes
        pos_fbk_vec(:,i) = fbk(1:3);
        v_l(i)= pl_u(1,1);
        v_r(i)= pl_u(1,2);

        % Record current planning input for 'minimize acceleration
        % constraint'
        u_ref= [v_l(i);v_r(i)];
        i=i+1;
        pl_status=1; 
        
        % Generate interpolated curve to query for controls
        ctrl_vel_ref_curve= makima(t_pl_vel,ctrl_ref); % TODO: Try PCHIP instead

        % Shift trajectory to initialize the next step of planning
        X0_pl = [X0_pl(2:end,:); X0_pl(end,:)];
    end
    
    % Whenever planner status goes to 1, new interpolated curve is utilized
    if pl_status==1
        count=0;
    else
        count= count+1;
    end

    % Create a relative time vector with lookahead for the controller
    t_ctrl= (linspace((count+1)*ctrl.Ts,(count+1+ctrl.N)*ctrl.Ts,ctrl.N+1))';
    
    ctrl_ref_curr= zeros(ctrl.nx,length(t_ctrl));

    % Interpolate the planned trajectory to control discretization
    ctrl_ref_curr(1:2,:)= ppval(ctrl_vel_ref_curve,t_ctrl); % v_l, v_r
    ctrl_ref_curr(3:4,:)= 0; % theta, thetaDot ref = 0

    % Reset planner status until next planning execution occurs
    pl_status=0; 
    
    try   
        % Try to solve MPC, see if it fails
        [tau_l(j),tau_r(j),~,per_loop_time_mod(j)]= ctrl.updateMPC(fbk_lin,ctrl_ref_curr);
        failure=false;
    catch ME
        failure=true;
        % If it fails, record failure and exit simulation
        break;
    end
    
    % Record control effort for metric
    control_effort(j)= norm(([tau_l(j);tau_r(j)]-[tau_l_prev;tau_r_prev]),2);

    tau_l_prev=tau_l(j);
    tau_r_prev=tau_r(j);

    % Update m_p(k) and l_p(k)    
    robot.m_p=master.m_vec(i);
    robot.l=master.l_vec(i);
    robot.j_psi= varyJpsi(robot.m_p,robot.l,fbk_lin(3));
    % Simulate nonlinear plant
    fbk= robot.PropagateRobotDynamics(fbk,[tau_l(j);tau_r(j)]);

    % Stage feedback
    fbk_vec(:,j+1) = fbk;
end

pl_loop_time = toc(pl_loop);  
ss_error = norm((fbk(1:2)-sim.xf(1:2)),2);

pl_loop_time/j;