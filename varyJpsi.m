function delta_j_psi = varyJpsi(m_p,l_p,theta)
    %inputs are mass, length, pitch angle and choice for linear (0) or nonlinear (1) varying function
        alpha = 0.04; % Example value
        beta = 0.07; % Example value
        gamma = 0.02; % Example value
        
        delta_j_psi= alpha*l_p + beta*m_p + gamma*theta;  
end

