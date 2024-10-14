fbk=sim.x0;
fbk_vec(:,1) = fbk;

% two inputs for robot at planner level
u0_pl = zeros(pl.N,2);       
% initialization of the kinematic states decision variables
X0_pl = repmat(sim.x0(1:pl.nx),1,pl.N+1)'; 

X_MRAC_bar_vec(:,1)= fbk(4:7);
X_MRAC_vec(:,1)=fbk(4:7);
state_prop_lin(:,1)= fbk(4:7);
err_mrac_vec(:,1)= X_MRAC_vec(:,1)-X_MRAC_bar_vec(:,1);
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
ctrl_count=0;
pl_loop = tic;
for j=1:K
    
    if(norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2) < 0.009)
        error_ego(j)= norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2);
        break;
    end
    
    
    for iter=1:sim.obs_num
     obs_mapping(iter,j) =norm(fbk(1:2)-[sim.obs_x(iter);sim.obs_y(iter)],2);
    end
    error_ego(j)= norm((fbk(1:pl.nxo)-sim.xf(1:pl.nxo)),2);
    t(j) = (j-1)*ctrl.Ts;
   
    fbk_lin= [fbk(4:7)];
    
    if(rem(j,pl_exec_freq)==1) %% Planning Cycle
        X0_pl(1,:) = fbk(1:pl.nx)';
        QT= computeTerminalCost(sim.xf(1:pl.nxo),pl_u(end,1),pl_u(end,2),pl.Q,pl.R);
        pl_args.p   = [fbk(1:pl.nx);sim.xf(1:pl.nxo);u_ref;QT(:)];
        
        pl_args.x0  = [reshape(X0_pl',pl.nx*(pl.N+1),1);reshape(u0_pl',pl.nu*pl.N,1)];
        pl_sol = pl_solver('x0', pl_args.x0, 'lbx', pl_args.lbx, 'ubx', pl_args.ubx,...
            'lbg', pl_args.lbg, 'ubg', pl_args.ubg,'p',pl_args.p);

        % get inputs only from the solution
        pl_u = reshape(full(pl_sol.x(pl.nx*(pl.N+1)+1:end))',pl.nu,pl.N)';
        % get solution TRAJECTORY for plotting purposes
        pl_st= reshape(full(pl_sol.x(1:pl.nx*(pl.N+1)))',pl.nx,pl.N+1)';
        pl_rec(:,1:pl.nx,i)= pl_st(:,1:3);

        X0_pl = reshape(full(pl_sol.x(1:pl.nx*(pl.N+1)))',pl.nx,pl.N+1)'; % get solution TRAJECTORY
        
        ctrl_ref= pl_u';

        % Shift trajectory to initialize the next step
             
        
         % record fbk for plotting purposes
        pos_fbk_vec(:,i) = fbk(1:3);
        v_l(i)= pl_u(1,1);
        v_r(i)= pl_u(1,2);
    
        u_ref= [v_l(i);v_r(i)];
        i=i+1;
        pl_status=1; 
        
        ctrl_vel_ref_curve= makima(t_pl_vel,ctrl_ref); % TODO: Try PCHIP instead
 
        X0_pl = [X0_pl(2:end,:); X0_pl(end,:)];
    end
    
    % Whenever planner status goes to 1, our start
    if pl_status==1
        count=0;
    else
        count= count+1;
    end

    % Create a relative time vector with lookahead for the controller
    t_ctrl= (linspace((count+1)*ctrl.Ts,(count+1+ctrl.N)*ctrl.Ts,ctrl.N+1))';
    
    ctrl_ref_curr= zeros(ctrl.nx,length(t_ctrl));

    % Interpolate the planned trajectory to control discretization
    ctrl_ref_curr(1:2,:)= ppval(ctrl_vel_ref_curve,t_ctrl);
    ctrl_ref_curr(3:4,:)= 0;
    % Reset planner status until next planning execution occurs
    if pl_status
        try   
            [tau_l_sol,tau_r_sol,predictions,per_loop_time_mod(j)]= ctrl.updateMPC(fbk_lin,ctrl_ref_curr);
            failure=false;
            ctrl_count=count;
        catch ME
            failure=true;
            cascaded_adaptive.abs_failure=true;
            break;
        end
    end
    
    pl_status=0;     
    %% IMPLEMENT MRAC
    
    if ctrl_count==length(tau_l_sol)
        break;
    end
    X_MRAC_bar_vec(:,j+1)= predictions((ctrl_count*4)+1:(ctrl_count*4)+4);
    tau_l(j)=tau_l_sol(ctrl_count+1);
    tau_r(j)=tau_r_sol(ctrl_count+1);
    ctrl_count=ctrl_count+1;
    
    taus= [tau_l(j);tau_r(j)];
    taus= taus+mrac.Ke'*err_mrac_vec(:,j);
    tau_l_mrac(j)=min(11.5, max(-11.5, taus(1)));
    tau_r_mrac(j)=min(11.5, max(-11.5, taus(2)));

    control_effort(j)= norm(([tau_l_mrac(j);tau_r_mrac(j)]-[tau_l_prev;tau_r_prev]),2);    
    tau_l_prev=tau_l_mrac(j);
    tau_r_prev=tau_r_mrac(j);
    
    robot.m_p=master.m_vec(i);
    robot.l=master.l_vec(i);
    robot.j_psi= varyJpsi(robot.m_p,robot.l,fbk_lin(3));
    fbk= robot.PropagateRobotDynamics(fbk,[tau_l_mrac(j);tau_r_mrac(j);]);
    
    X_MRAC_vec(:,j+1)= fbk(4:7);
    err_mrac_vec(:,j+1) = X_MRAC_vec(:,j+1)-X_MRAC_bar_vec(:,j+1);
    mrac=mrac.updateMRACgains(err_mrac_vec(:,j+1));
    gains_MRAC_vec(j+1).Ke= mrac.Ke;
    fbk_vec(:,j+1) = fbk;
end
pl_loop_time = toc(pl_loop);  
ss_error = norm((fbk(1:2)-sim.xf(1:2)),2);

pl_loop_time/j;