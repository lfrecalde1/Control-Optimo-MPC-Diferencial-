%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXCONTROL DE TRAYECTORIA DE UNA PLATAFORMA MOVILXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%% PARAMETROS DE TIEMPO
clc,clear all,close all;
ts=0.1;
tfinal=50;
to=0;
t=[to:ts:tfinal+ts];
load('DINAMICA_PLATAFORMA.mat');
%% DISTANCIAS DE LA PALTAFORMA MOVIL
a=0.15;
l=[a]';
%% POSICIONES INICIALES 
hx(1)=1;
hy(1)=0.5;
phi(1)=180*pi/180;
%% VELOCIDADES INICIALES 
u(1) = 0;
w(1) = 0;
%% VELOCIDADES ESTIMADAS
u_e(1)=0;
w_e(1)=0;

%% CINEMATICA DIRECTA 
hx(1)=hx(1)+a*cos(phi(1));
hy(1)=hy(1)+a*sin(phi(1));
hxp(1)=0;
hyp(1)=0;
%% TRAYECTORIA DESEADAS
path=[0,0;2,0;2,2;0,2;0,0;2,2;4,2];
path=[hx(1),hy(1);path];
[trayectoria,trayectoria_p]=PTP(path,to,tfinal,ts);
hxd=1*sin(0.3*t);
hyd=0.1*t;
hxdp=1*0.3*cos(0.3*t);
hydp=0.1*ones(1,length(t));

hxd=1*sin(0.3*t);
hyd=0.1*t;
hxdp=1*0.3*cos(0.3*t);
hydp=-1*0.3*sin(0.3*t);

hd=[hxd;hyd];
%% RESTRICCION PARA LAS ACCIONES DE CONTROL
lb = [-0.5,-2.5]';
ub = [ 0.5, 2.5]';

z0=[u(1),w(1)]';


%% CONFIGURACION DEL METODO DE OPTIMIZACION A UTILIZAR
options = optimset('Algorithm','sqp','Display','off');
N=10;



%Obj=[-2 + (2-(-2)).*rand(1,length(t)-100);0 + (5-(0)).*rand(1,length(t)-100)];
Obj=[0 -1;2.1,1.5];

Obj=[hxd(1,1:40:length(t));hyd(1,1:40:length(t))];

% Obj=[-1+1*sin(0.02*t(1:30:length(t)));1.5+1*cos(0.02*t(1:30:length(t)))];
for k=1:length(t)-N
    tic
    hxe(k)=hxd(k)-hx(k);
    hye(k)=hyd(k)-hy(k);
    
    %% GANANCIA PARA OPTIMIZADOR
    H=[1,0;...
        0,1];
    R=1*eye(2);
    
     %% VECTORES PARA FORMA GENERAL
     hdp=[hxdp(k) hydp(k)]';
     h =[hx(k),hy(k)]';
     hp=[hxp(k),hyp(k)]';
     q=[0 phi(k)]'; 
     v_real=[u(k) w(k)]';
%      v_estimado=[u_e(k) w_e(k)]';
     
     %% CONTROLADOR BASADO EN OPTIMIZACION
     f_obj = @(z) movil_optimo(z,H,R,hd,h,q,l,ts,N,k,v_real,x,Obj);
     qref = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
 
     %% VELOCIDADES CINEMATICAS O VELOCIDADES DESEADAS PARA EL BLOQUE DE COMPENSACI�N DIN�MICA
     uref_c(k)=qref(1);
     wref_c(k)=qref(2);
     
     %% ERRORES DE VELOCIDAD VREF
   
     vref =[uref_c(k) wref_c(k)]';

     movil = MOVIL_DINAMICA(vref,v_real,q,ts,x);
     
     %% VELOCIDADES DEL ROBOT
     u(k+1)=movil(1);
     w(k+1)=movil(2);
     
%      movil_estimado= MOVIL_DINAMICA(vref,v_estimado,q,ts,chi_estimado(:,k+1));
%      %% VELOCIDADES DEL ROBOT
%      u_e(k+1)=movil_estimado(1);
%      w_e(k+1)=movil_estimado(2);
     
     
     
     %% POSICIONES DEL ROBOT 
     phi(k+1)=movil(3); 
     %% CINEMATICA DIRECTA 
     hxp(k+1)=u(k+1)*cos(phi(k+1))-a*w(k+1)*sin(phi(k+1));
     hyp(k+1)=u(k+1)*sin(phi(k+1))+a*w(k+1)*cos(phi(k+1));
     hx(k+1)=ts*hxp(k+1)+hx(k);
     hy(k+1)=ts*hyp(k+1)+hy(k);
     
     z0 = [uref_c(k),wref_c(k)]';
     
     %chi0=chi_estimado(:,k+1);
     
     uref_c1=uref_c(k);
     wref_c1=wref_c(k);
     toc
     
end
largo=0.4;
ancho=0.3;
% SIMULACION(a,largo,ancho,hx,hy,hxd,hyd,phi,ts);
figure
plot(hxd(1,1:length(hx)),hyd(1,1:length(hx)),'c'); hold on
plot(hx,hy,'k'); grid
plot(Obj(1,:),Obj(2,:),'*r'); grid on
title('$\textrm{Trayectoria Descrita y Trayectoria Deseada}$','Interpreter','latex','FontSize',13);
legend({'$\mathbf{\eta}_{p_{des}}$','$\mathbf{\eta}_{p}$'},'Interpreter','latex','FontSize',13);
xlabel('$x[m]$','Interpreter','latex','FontSize',13); ylabel('$y[m]$','Interpreter','latex','FontSize',13);
figure
subplot(2,1,1)

    plot(t(1:length(hxe)),hxe,'r'); hold on
    plot(t(1:length(hxe)),hye,'g'); hold on
    grid on;
    legend({'$\tilde{x_p}$','$\tilde{y_p}$'},'Interpreter','latex','FontSize',13);
    title('$\textrm{Errores de Posicion}$','Interpreter','latex','FontSize',13);
     xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Error}[m]$','Interpreter','latex','FontSize',13);
    
subplot(2,1,2)
    plot(t(1,1:length(hxe)),ue,'k'); hold on
    plot(t(1,1:length(hxe)),we,'c'); hold on
    grid on;
    legend({'$\tilde\mu$','$\tilde{\dot\psi_{p}}$'},'Interpreter','latex','FontSize',13);
    title('$\textrm{Errores de Velocidad}$','Interpreter','latex','FontSize',13);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Error}[m/s][rad/s]$','Interpreter','latex','FontSize',13);

figure
    subplot(2,1,1)
    plot(t(1:length(u)),u,'r'); hold on
    plot(t(1:length(u)),w,'c'); hold on
    plot(t(1:length(uref_c)),uref_c,'--r'); hold on
    plot(t(1:length(uref_c)),wref_c,'--c'); hold on
%     plot(t(1:length(uref_f)),uref_f,'--k'); hold on
%     plot(t(1:length(uref_c)),vrefp_w,'--m'); hold on
    grid on
    title('$\textrm{Velocidades a la Salida del Robot y Velocidades de Control Cinematico}$','Interpreter','latex','FontSize',13);
    legend({'$\mu$','$\dot\psi_{p}$','$\mu_{ref_{c}}$','$\dot\psi_{p_{ref_{c}}}$'},'Interpreter','latex','FontSize',13);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Velocidad}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
    subplot(2,1,2)

    plot(t(1,1:length(uref)),uref,'m'); hold on
    plot(t(1,1:length(wref)),wref,'c'); hold on
    grid on
    title('$\textrm{Velocidades de Compensacion Dinamica}$','Interpreter','latex','FontSize',13);
    legend({'$\mu_{ref}$','$\dot\psi_{p_{ref}}$'},'Interpreter','latex','FontSize',13);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Velocidad}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
    
    