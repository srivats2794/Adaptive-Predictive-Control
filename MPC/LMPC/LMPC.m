classdef LMPC
    %LMPC OSQP based linear MPC for the self balancing robot
   
    properties
       Ts;ld;
       x_min; x_max; u_min; u_max;
       Q; QT; R; N; nx; nu;
       Ad;Bd;
       solver; verbose;
    end
    
    methods
        function obj = LMPC(Ts,ld,mpc_params,sys_params,verbose)
            %LMPC Construct an instance of the LMPC class
            obj.Ts=Ts;obj.ld=ld;
            obj.N=mpc_params.N;
            obj.Q=mpc_params.Q; 
            
            obj.R=mpc_params.R;
            obj.x_max=mpc_params.x_max; obj.x_min=mpc_params.x_min;
            obj.u_min=mpc_params.tau_min;obj.u_max=mpc_params.tau_max;
            obj=setupSystem(obj,sys_params);
            [obj.nx,obj.nu]=size(obj.Bd);
            [obj.QT,~,~] = idare(obj.Ad,obj.Bd,obj.Q,obj.R);  
            %obj= computeTerminalCost(obj);
            %obj.QT=mpc_params.QT;
            obj.verbose= boolean(verbose);
            obj=setupMPC(obj); 
        end
        
        function this=setupSystem(this,sys)
            % This function computes the A and B matrix for the control design

            %% Predefining some repeated cluster terms
            Me= sys.m_b+2*sys.m_w+sys.m_p+((2*sys.j_w)/(sys.r_w^2));
            Je= sys.j_m_th+sys.j_p_th;

            %% Predifining some big constants to avoid typos
            p01= sys.m_p*sys.l*sys.l+Je;
            p02= sys.m_p*9.8056*sys.l;
            p03= sys.m_p*sys.l;
            p04= sys.m_p*sys.m_p*sys.l*sys.l;

            p1= (-p02*Me)/(p04-p01*Me);
            p2= p03/(sys.r_w*p04-Me*sys.r_w*p01);
            p3= (-p04*9.8056)/(Me*p01-p04);
            p4= p01/(Me*sys.r_w*p01-p04*sys.r_w);
            p5= (sys.w*sys.w*sys.r_w)/ ...
                ((2*sys.j_psi*sys.r_w*sys.r_w)+ ...
                ((sys.j_w+sys.r_w*sys.r_w*sys.m_w)*sys.w*sys.w));

            %% Order documentation
            % States order -> x_l_dot,x_r_dot,theta,theta_Dot
            % Inputs order -> tau_l,tau_r

            %% System matrices continuous time

            A= [0 0 p3 0;
                0 0 p3 0;
                0 0 0  1;
                0 0 p1 0];


            B= [p4+p5 p4-p5;
                p4-p5 p4+p5;
                0     0;
                p2    p2;];

            C= eye(4);

            D = zeros(4,2);

            %% Discretization
            [this.Ad,this.Bd,~,~]=ssdata(c2d(ss(A,B,C,D),this.Ts)); 
        end

        function obj = computeTerminalCost(obj)
            G0=obj.Bd * (obj.R \ obj.Bd');
            H0=obj.Q; 
            % Initialize matrices
            A_k = obj.Ad;
            G_k = G0;
            H_k = H0;

            % Initialize the loop variables
            norm_diff = 1e-16;
            i=0;
            while norm_diff >= 1e-8
                % Compute the Cholesky decomposition of I + G_k H_k
                I_plus_GH = eye(size(G_k * H_k)) + G_k * H_k;
                %L = chol(I_plus_GH, 'lower');

                % Solve for inv_term using back-substitution
                % inv_term = (L^T \ (L \ I))
                %inv_term = L' \ (L \ eye(size(L)));

                % Update A_{k+1}
                A_k_plus_1 = A_k * (I_plus_GH \ A_k);

                % Update G_{k+1}
                G_k_plus_1 = G_k + A_k * (I_plus_GH \ G_k) * A_k';

                % Update H_{k+1}
                H_k_plus_1 = H_k + A_k' * H_k * (I_plus_GH \ A_k);

                % Compute the relative change
                norm_diff = norm(H_k_plus_1 - H_k, 'fro') / norm(H_k_plus_1, 'fro');

                % Update matrices for next iteration
                A_k = A_k_plus_1;
                G_k = G_k_plus_1;
                H_k = H_k_plus_1;
                i=i+1;
            end

            % Return the final H matrix
            obj.QT = H_k;
        end

        function this=setupMPC(this)
            P = blkdiag( kron(speye(this.N), this.Q), this.QT, kron(speye(this.N), this.R) );
            q = [repmat(-this.Q*zeros(this.nx,1), this.N,1); -this.QT*zeros(this.nx,1);zeros(this.N*this.nu, 1)];
            Ax = kron(speye(this.N+1), -speye(this.nx)) + kron(sparse(diag(ones(this.N, 1), -1)), this.Ad);
            Bu = kron([sparse(1, this.N); speye(this.N)], this.Bd);
            Aeq = [Ax, Bu];
            leq = [-zeros(this.nx,1); zeros(this.N*this.nx, 1)];
            ueq = leq;
            % - input and state constraints
            Aineq = speye((this.N+1)*this.nx + this.N*this.nu);
            lineq = [repmat(this.x_min, this.N+1, 1); repmat(this.u_min*ones(this.nu,1), this.N, 1)];
            uineq = [repmat(this.x_max, this.N+1, 1); repmat(this.u_max*ones(this.nu,1), this.N, 1)];
            % - OSQP constraints0
            A = [Aeq; Aineq];
            l = [leq; lineq];
            u = [ueq; uineq];

            % Create an OSQP object
            this.solver = osqp;

            % Setup workspace
            this.solver.setup(P, q, A, l, u, 'warm_start', true,'verbose',this.verbose);
        end
    
        function [tau_l,tau_r,prediction,run_time]=updateMPC(this,feedback,reference)
            %% Transforming measurement to controller states
            
            % Controller States -> X_L,X_R, X_L_Dot, X_R_Dot theta, thetaDot
            x0= feedback;
            
            %% State part of the objective function
            q_new1 = reshape((-this.Q*reference(:,1:end-1)),this.nx*(this.N),1); 
            q_new2 = -this.QT*reference(:,this.N+1);
            q_new3 = zeros(this.N*this.nu,1);

            q_new= [q_new1;q_new2;q_new3];
            
            %% Equality constraints
            leq = [-x0; zeros(this.N*this.nx, 1)];
            ueq = leq;
            
            %% Inequality constraints -> input and state constraints
            lineq = [repmat(this.x_min, this.N+1, 1); repmat(this.u_min*ones(this.nu,1), this.N, 1)];
            uineq = [repmat(this.x_max, this.N+1, 1); repmat(this.u_max*ones(this.nu,1), this.N, 1)];
            
            % - OSQP constraints
            l_new = [leq; lineq];
            u_new = [ueq; uineq];

            this.solver.update('q', q_new, 'l', l_new, 'u', u_new);
            
            %%%%%%% solve %%%%%%%
            res          = this.solver.solve();
            run_time=res.info.run_time;
            % check solver status
            if ~strcmp(res.info.status, 'solved')
                error('OSQP did not solve the problem!')
            end

            %%%%%%% apply first control input to the plant %%%%%%%
            ins        = res.x((this.N+1)*this.nx+1:(this.N+1)*this.nx+this.nu);
            tau_l= ins(1);
            tau_r= ins(2);
           
            % - MPC solution: x(k+1)
            prediction       = res.x(this.ld*this.nx+1:this.ld*this.nx+this.nx);
        end
    end
end

