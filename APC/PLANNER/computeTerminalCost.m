function [QT] = computeTerminalCost(ref_states,v_l,v_r,Q,R)
    Ts=0.1;
    L=0.44;
    v=(v_l+v_r)/2;
    psi_ref= ref_states(3);
    A = [1, 0, -v * sin(psi_ref) * Ts;
         0, 1, v * cos(psi_ref) * Ts;
         0, 0, 1];
    
    B = [cos(psi_ref) * Ts / 2, cos(psi_ref) * Ts / 2;
         sin(psi_ref) * Ts / 2, sin(psi_ref) * Ts / 2;
         -Ts / L, Ts / L]; % 
    
    % Compute the terminal cost matrix P
    [QT,~,~] = idare(A,B,Q,R);
end

