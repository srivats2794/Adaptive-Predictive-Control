classdef MRAC_CONTROLLER
    % A class implementation that encapsulates all the MRAC controller properties and methods 
    
    
    properties
       gamma_st; 
       gamma_err;
       gamma_in; 
       Q 
       A,Ad
       B,Bd
       P
       Kin; Ky;Ke; Ky0;
       lambda;
       Ts
    end
    
    methods
        function obj = MRAC_CONTROLLER(params)
            %MRAC_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj.gamma_err= params.gamma_err;
            obj.Q= params.Q;
            obj.A= params.A;
            obj.B= params.B;
            obj.Ad= params.Ad;
            obj.Bd= params.Bd;
            obj.P=params.P;
            obj.Ts= params.Ts;
            obj.Ke= params.Ke;
            obj.lambda= params.lambda;
        end
        
        function this = updateMRACgains(this,err)
            
            err_norm= norm(err,1);

            Ke_dot= -this.gamma_err*(err*err')*this.P*this.B-this.lambda*err_norm*this.Ke;
            this.Ke=Ke_dot*this.Ts+this.Ke;
        end

    end
end

