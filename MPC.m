function [qref] = MPC(z0,Q,R,hd,h,q,l,ts,N,k,v_real,x,Obj,bandera,LB,UB,options)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% options.Display =d;
f_obj = @(z) movil_optimo_2(z,Q,R,hd,h,q,l,ts,N,k,v_real,x,Obj,bandera);
qref = fmincon(f_obj,z0,[],[],[],[],LB,UB,[],options);
end

