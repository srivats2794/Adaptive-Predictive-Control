function params = setupMRAC(sys,Ts,mpc)
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

    params.A= [0 0 p3 0;
               0 0 p3 0;
               0 0 0  1;
               0 0 p1 0];
    
    
    params.B= [p4+p5 p4-p5;
               p4-p5 p4+p5;
                0     0;
               p2    p2;];
    
    params.C= eye(4);
    
    params.D = zeros(4,2);

    [params.Ad,params.Bd,~,~] = ssdata(c2d(ss(params.A,params.B,params.C,params.D),Ts));
    factor= 9e-2;
    params.lambda= 0.9;

    params.gamma_err = factor*diag([1;1;1;1]);

    [params.Q,params.Ky0,~]= icare(params.A,params.B,mpc.Q,mpc.R);
    params.P= eye(4);
    params.Ke= zeros(4,2);
    params.Ke=[-0.000159308696400636,-0.000159305231805153;-0.000318917619519561,-0.000318923858602919;-0.000162088135536871,-0.000162088969440858;-0.000107380945473005,-0.000107381537868504];
    params.Ts= Ts;
end

