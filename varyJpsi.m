function delta_j_psi = varyJpsi(m_p,l_p,theta,choice)
    %inputs are mass, length, pitch angle and choice for linear (0) or nonlinear (1) varying function

    if choice
        alpha = 0.05; % Example value
        beta = 0.1; % Example value
        gamma = 0.02; % Example value
        delta = 0.01; % Example value
        epsilon = 0.02; % Example value
        zeta = 0.01; % Example value

        delta_j_psi= alpha*l_p.^2 + beta*m_p.^2 + gamma*theta.^2 + ...
                       delta*l_p.*m_p + epsilon*l_p.*theta + zeta*m_p.*theta;       
    else
        alpha = 0.05; % Example value
        beta = 0.1; % Example value
        gamma = 0.02; % Example value
        
        delta_j_psi= alpha*l_p + beta*m_p + gamma*theta;  
    end
end

