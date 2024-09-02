classdef ROBOT < handle
    % A class that sets up the robot dynamics plant with
    % parameters and the neccessary methods needed to simulate it.
    properties
        w;                              
        r_w;                            
        j_m_th;                        
        j_p_th;                       
        j_w;                            
        m_b;                        
        m_w;                            
        m_p;                           
        l;                               
        j_psi;                        
        g;
        Ts;
        plant_choice;
        A;
        B;
    end
    
    methods
        function obj= ROBOT(robot_params,simSampleTime,plant_choice)
            obj.w = robot_params.w;                               % Width
            obj.r_w = robot_params.r_w;                              % Radius of Wheel
            obj.j_m_th=robot_params.j_m_th;                           % Base pitch moment of inertia
            obj.j_p_th= robot_params.j_p_th;                          % Pendulum pitch moment of interia
            obj.j_w= robot_params.j_w;                             % Rotational inertia of wheel
            obj.m_b= robot_params.m_b;                            % Base mass
            obj.m_w= robot_params.m_w;                              % Wheel mass
            obj.m_p=robot_params.m_p;                                  % Pendulum mass
            obj.l= robot_params.l;                                % Pendulum length
            obj.j_psi=  robot_params.j_psi;                           % Base yaw moment of inertia
            obj.g= 9.8056;
            obj.Ts= simSampleTime;
            obj.plant_choice=plant_choice;
            lin_sys_setup(obj);
        end
        
        function [statesDot]= RobotDynamics(this,states,ins)

            tau_l= ins(1);
            tau_r= ins(2);

            % State order X Y Psi V_L V_R Theta Theta_Dot 
            xDot_l= states(4);
            xDot_r= states(5);
            xDot= (xDot_l+xDot_r)/2;
            psiDot=(xDot_l-xDot_r)/this.w;
            psi= states(3);
            theta= states(6);
            thetaDot= states(7);
            
            p1= this.m_b+(2*this.m_w)+this.m_p+((2*this.j_w)/(this.r_w^2));
            p2= this.m_p*this.l*this.l+this.j_m_th+this.j_p_th;
            p3= this.m_p*this.l;
            Me= p1;
            
            if this.plant_choice            
               
               xDDot_num= (tau_l/this.r_w)+(tau_r/this.r_w)...
                         +(p3*(thetaDot^2)*sin(theta))...
                         -(((p3^2)*this.g*sin(theta)*cos(theta))/p2);
               xDDot_den= Me-(((p3*cos(theta))^2)/p2);

               xDDot= xDDot_num/xDDot_den;

               th_num= ((p3*cos(theta)/p1)*((tau_l/this.r_w)+...
                   (tau_r/this.r_w)+(p3*thetaDot^2*sin(theta))))...
                   -p3*this.g*sin(theta);
               
               th_den= (((p3*cos(theta))^2)/p1)-p2;

               thetaDDot= th_num/th_den;

               psiDDot= (this.w*((tau_l-tau_r)/this.r_w))/...
                   (this.j_psi+...
                   (((this.j_w+(this.r_w^2)*this.m_w)*this.w*this.w)/(2*this.r_w*this.r_w)));
               xDDot_l= xDDot+(this.w/2)*psiDDot;
               xDDot_r= xDDot-(this.w/2)*psiDDot;    
            else
              % State order linear v_l v_r theta thetaDot
              X_lin= [xDot_l;xDot_r;theta;thetaDot];
              XDot_lin= this.A*X_lin+this.B*ins;
             
              thetaDot= XDot_lin(3);
              xDDot_l= XDot_lin(1);
              xDDot_r= XDot_lin(2);
              thetaDDot= XDot_lin(4);          
            end
             % State order X Y psi v_l v_r theta thetaDot
            statesDot= [xDot*cos(psi);xDot*sin(psi);psiDot;xDDot_l;xDDot_r;thetaDot;thetaDDot];
        end
        
        function [states_next,statesDot]= PropagateRobotDynamics(this,states,inputs)
            if this.Ts<0.01
                % Euler
                statesDot=RobotDynamics(this,states,inputs);
                states_next= statesDot*this.Ts+states;
            else
                % RK4
                k1 = RobotDynamics(this,states,inputs);
                k2 = RobotDynamics(this,states + this.Ts/2*k1, inputs);
                k3 = RobotDynamics(this,states + this.Ts/2*k2, inputs);
                k4 = RobotDynamics(this,states + this.Ts*k3, inputs);
                statesDot= 1/6*(k1 +2*k2 +2*k3 +k4);
                states_next=states + this.Ts*statesDot;
            end
        end
    
        function lin_sys_setup(this)
            %% Predefining some repeated cluster terms
            p1= this.m_b+(2*this.m_w)+this.m_p+((2*this.j_w)/(this.r_w^2));
            p2= this.m_p*this.l*this.l+this.j_m_th+this.j_p_th;
            p3= this.m_p*this.l;
            
            f1= (-p3*p1*this.g)/((p3^2)-p2*p1);
            f2= p3/((this.r_w*p3*p3)-(p1*p2*this.r_w));
            g1= (-p3*p3*this.g)/((p1*p2)-(p3^2));

            m1= p2/((p1*p2*this.r_w)-(p3*p3*this.r_w));
            m2= (this.w*this.w*this.r_w)/...
                ((2*this.j_psi*this.r_w*this.r_w)+...
                ((this.j_w+this.r_w*this.r_w*this.m_w)*this.w*this.w));

            g2= m1+m2;
            g3= m1-m2;

            % State order linear-> psi theta v psiDot thetaDot

            this.A= [0   0   g1  0;
                     0   0   g1  0;
                     0   0   0   1; 
                     0   0   f1  0];

            this.B= [ g2          g3;
                      g3          g2;
                      0           0;
                      f2         f2];
        end
    end
end
