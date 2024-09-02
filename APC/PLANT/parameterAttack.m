function [params] = parameterAttack(params,percent)
    params.j_m_th=params.j_m_th+(percent.j_m_th/100)*params.j_m_th;                        
    params.j_p_th= params.j_p_th+(percent.j_p_th/100)*params.j_p_th;                                                 
    params.m_p=params.m_p+(percent.m_p/100)*params.m_p;                                
    params.l= params.l+(percent.l/100)*params.l;                               
    params.j_psi= params.j_psi+(percent.j_psi/100)*params.j_psi;                           
end


