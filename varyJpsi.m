function delta_j_psi = varyJpsi(m_p,l_p,theta)
    %inputs are mass, length, pitch angle and choice for linear (0) or nonlinear (1) varying function
        alpha = 0.05; 
        beta = 0.1; 
        gamma = 0.5; 
        
        delta_j_psi= alpha*l_p + beta*m_p + gamma*theta;  
end

