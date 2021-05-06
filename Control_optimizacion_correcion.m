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
hx(1)=0;
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

hxd=1*sin(0.3*t);
hyd=0.1*t;
hxdp=1*0.3*cos(0.3*t);
hydp=-1*0.3*sin(0.3*t);


% hxd=0*ones(1,length(t));
% hyd=1*ones(1,length(t));
% hxdp=0*ones(1,length(t));
% hydp=0*ones(1,length(t));
hd=[hxd;hyd];

%% Este valor cambiar si desea cambiar el horizonte de predciccion si se aumenta se demora mas optimizando
N=4;

%% RESTRICCION PARA LAS ACCIONES DE CONTROL
lb = [-0.5,-2.5]';
ub = [ 0.5, 2.5]';
LB=[];
UB =[];
for index=1:N-1
    LB=[LB;lb];
    UB=[UB;ub];
end

%% CONFIGURACION DEL METODO DE OPTIMIZACION A UTILIZAR
options = optimset('Algorithm','sqp','Display','off');


%% generacion de la solucion iniciale del sistema
z0=[u(1)*ones(1,N-1);...
    w(1)*ones(1,N-1)];


%% Generacion de los objetos a evadir
Obj=[hxd(1,1:40:length(t));hyd(1,1:40:length(t))];


%% Valores para matriz de ganancia
Q=1;
R=0.0005; %% Si se aumenta esa las acciones de control son mas suaves para puede que no se llega a erroes de cero

%% bandera para objetos
bandera=0;  %% 0 sin objetos 1 con objetos
for k=1:length(t)-N
    tic;
    hxe(k)=hxd(k)-hx(k);
    hye(k)=hyd(k)-hy(k);
    
 
     %% VECTORES PARA FORMA GENERAL
     hdp=[hxdp(k) hydp(k)]';
     h =[hx(k),hy(k)]';
     hp=[hxp(k),hyp(k)]';
     q=[0 phi(k)]'; 
     v_real=[u(k) w(k)]';

     
     %% CONTROLADOR BASADO EN OPTIMIZACION
     f_obj = @(z) movil_optimo_2(z,Q,R,hd,h,q,l,ts,N,k,v_real,x,Obj,bandera);
     qref = fmincon(f_obj,z0,[],[],[],[],LB,UB,[],options);
 
     %% VELOCIDADES CINEMATICAS O VELOCIDADES DESEADAS PARA EL BLOQUE DE COMPENSACI�N DIN�MICA
     uref_c(k)=qref(1,1);
     wref_c(k)=qref(2,1);
     
     %% ERRORES DE VELOCIDAD VREF
   
     vref =[uref_c(k) wref_c(k)]';

     movil = MOVIL_DINAMICA(vref,v_real,q,ts,x);
     
     %% VELOCIDADES DEL ROBOT
     u(k+1)=movil(1);
     w(k+1)=movil(2);
     
     
     ue(k)=uref_c(k)-u(k);
     we(k)=wref_c(k)-w(k);
     
     
     
     %% POSICIONES DEL ROBOT 
     phi(k+1)=movil(3); 
     %% CINEMATICA DIRECTA 
     hxp(k+1)=u(k+1)*cos(phi(k+1))-a*w(k+1)*sin(phi(k+1));
     hyp(k+1)=u(k+1)*sin(phi(k+1))+a*w(k+1)*cos(phi(k+1));
     hx(k+1)=ts*hxp(k+1)+hx(k);
     hy(k+1)=ts*hyp(k+1)+hy(k);
     
     z0 = [qref(1,:);...
           qref(2,:)];
     
     t_sample(k)=toc;
     
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
    
   figure
    plot(t(1:length(t_sample)),t_sample,'r'); hold on

    grid on
    title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',13);
    legend({'$t_{sample}$'},'Interpreter','latex','FontSize',13);
    xlabel('$\textrm{Tiempo }[s]$','Interpreter','latex','FontSize',13); ylabel('$\textrm{Velocidad}[m/s][rad/s]$','Interpreter','latex','FontSize',13);
    
    
    emsx =hxe*hxe'
    emsy =hye*hye'
     