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
            obj.gamma_st = params.gamma_st;
            obj.gamma_err= params.gamma_err;
            obj.gamma_in= params.gamma_in;
            obj.Q= params.Q;
            obj.A= params.A;
            obj.B= params.B;
            obj.Ad= params.Ad;
            obj.Bd= params.Bd;
            obj.P=params.P;
            obj.Ts= params.Ts;
            obj.Ke= params.Ke;
            obj.Kin= params.Kin;
            obj.Ky= params.Ky;
            obj.Ky0 = params.Ky0;
            obj.lambda= params.lambda;
        end
        
        function this = updateMRACgains(this,meas,err,in)
            
            err_norm= norm(err,2);

            Ky_dot= this.gamma_st*meas*err'*this.P*this.B-this.lambda*err_norm*this.Ky;
            Kin_dot= this.gamma_in*in*err'*this.P*this.B-this.lambda*err_norm*this.Kin;
            Ke_dot= -this.gamma_err*(err*err')*this.P*this.B-this.lambda*err_norm*this.Ke;

            this.Kin= Kin_dot*this.Ts+this.Kin;
            this.Ky= Ky_dot*this.Ts+this.Ky;
            this.Ke=Ke_dot*this.Ts+this.Ke;
        end

    end
end

