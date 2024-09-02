function sys=setupRobotParams()
    sys.w = 0.44;                               % Width
    sys.r_w = 0.1;                              % Radius of Wheel
    sys.j_m_th=0.153;                           % Base pitch moment of inertia
    sys.j_p_th= 0.125;                          % Pendulum pitch moment of interia    
    sys.j_w= 0.013;                             % Rotational inertia of wheel
    sys.m_b= 15.747;                            % Base mass
    sys.m_w= 5.44;                              % Wheel mass
    sys.m_p=4;                                  % Pendulum mass
    sys.l= 0.53;                                % Pendulum length
    sys.j_psi= 0.576;                           % Base yaw moment of inertia
end
