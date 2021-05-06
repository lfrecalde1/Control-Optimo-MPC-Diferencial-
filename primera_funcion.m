function [trayectoria,trayectoria_p]=primera_funcion(path,to,tf,ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
vacio=0*ones(size(path));
t=to:ts:tf;
div=tf/(length(path)-1);
t_1=to:div:tf;
for index=1:length(t_1)
    location(index,:)=find((t>=t_1(index))&(t<=t_1(index)+ts));
end

factores=[];
for q=1:length(path)-1
    
    entrada=[path(q,1),path(q,2);...
             vacio(q,1),vacio(q,2);...
             path(q+1,1),path(q+1,2);...
             vacio(q+1,1),vacio(q+1,2)];
    
    valores=[1,t_1(q),t_1(q)^2,t_1(q)^3;...
         0,1,2*t_1(q),3*t_1(q)^2;...
         1,t_1(q+1),t_1(q+1)^2,t_1(q+1)^3;...
         0,1,2*t_1(q+1),3*t_1(q+1)^2];
    factores_1=inv(valores)*entrada;
    factores=[factores,factores_1];
    
end
k_1=1;
factores_dim=size(factores);
if((factores_dim(1,2))>2)
    for i_1=1:2:(factores_dim(1,2))-1 
     fac_x=factores(:,i_1);
     fac_y=factores(:,i_1+1);
         for j_ii=location(k_1,1):location(k_1+1,1)
            ecx(j_ii)=[1,t(j_ii),t(j_ii)^2,t(j_ii)^3]*fac_x;
            ecy(j_ii)=[1,t(j_ii),t(j_ii)^2,t(j_ii)^3]*fac_y;
            
            ecx_p(j_ii)=[0,1,2*t(j_ii),3*t(j_ii)^2]*fac_x;
            ecy_p(j_ii)=[0,1,2*t(j_ii),3*t(j_ii)^2]*fac_y;
           
         end
    k_1=k_1+1;
    end
else 
     for i_1=1:1:(factores_dim(1,2))-1
     fac_x=factores(:,i_1);
     fac_y=factores(:,i_1+1);
         for j_ii=location(k_1,1):location(k_1+1,1)
             
            ecx(j_ii)=[1,t(j_ii),t(j_ii)^2,t(j_ii)^3]*fac_x;
            ecy(j_ii)=[1,t(j_ii),t(j_ii)^2,t(j_ii)^3]*fac_y;
            
            ecx_p(j_ii)=[0,1,2*t(j_ii),3*t(j_ii)^2]*fac_x;
            ecy_p(j_ii)=[0,1,2*t(j_ii),3*t(j_ii)^2]*fac_y;
           
           
         end
    k_1=k_1+1;
    end   
end
trayectoria=[ecx;ecy];
trayectoria_p=[ecx_p;ecy_p];
end

