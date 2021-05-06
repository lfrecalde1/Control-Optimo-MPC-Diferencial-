function [f] = movil_optimo_2(z,Q,R,hd,h,q,l,ts,N,k,vreal,x,obj,bandera)
%1) Definici�n de variables
%a) Acciones de control (incognita)
vc = z ;      %z=[u,w,q1_p,q2_p,q3_p,q4_p]';

VC=[z(1,:)';z(2,:)'];
%b) Estados de control
th = q(2);


%c) Par�mtros del Manipulador M�vil
a = l(1);


L = [Fun(hd(:,k),h(:,1))];

u(1)=vreal(1);

w(1)=vreal(2);


T=[delta(vreal)];

r=0.2;

b=-0.14;

Fuerza=[evasion(obj,h(:,1),r,b)];

for i=1:N-1
    
    v_real=[u(i),w(i)]';
    
    estados=[0 th(i)]';
    
    vref=vc(:,i);
    
    movil = MOVIL_DINAMICA(vref,v_real,estados,ts,x);
    
    %% VELOCIDADES DEL ROBOT
    u(i+1)=movil(1);
    w(i+1)=movil(2);
    
    %2) Matriz Jacobiana
    j11 = cos(th(i));
    j12 = -a*sin(th(i));
    
    
    j21 = sin(th(i));
    j22 = +a*cos(th(i));
    
    
    
    
    J = [j11 j12;
        j21 j22];
    
    
    v=[u(i+1);w(i+1)];
    
    h(:,i+1)=ts*J*v+h(:,i);
    
    th(i+1) = th(i)+v(2)*ts;
    
    
    L=[L;(Fun(hd(:,k+i),h(:,i+1)))];
    
    T=[T;delta(v)];
    
    Fuerza=[Fuerza,evasion(obj,h(:,i+1),r,b)];
    
end

%% seccion para Seleccionar si se quiere o no evasion de obstaculos
%% Generacion de las matrices de ponderacion
Q=Q*eye(length(L));
R = R*eye(length(VC));
P = 0.008*eye(length(Fuerza));

%% Generaion de las funciones objetivo incluido objetos
%f=1*L'*Q*L+VC'*R*VC+Fuerza*P*Fuerza';

%% Generaion de las funciones solo trayectoria y acciones de control
% f=1*L'*Q*L+VC'*R*VC;

if bandera ==0
    f=1*L'*Q*L+VC'*R*VC;
else
    f=1*L'*Q*L+VC'*R*VC+Fuerza*P*Fuerza';
end

end
function [F] = Fun(hd,h)
alpha=1;    %Peso de errores de posici�n
he = hd-h;    %Errores de control
%     F = alpha*he'*H*he
F=he;
end

function [U] = delta(delta_u)
beta = 0.1;    %Peso de errores de posici�n
U = delta_u;
end
function [obstaculo]=evasion(obj,h,r,b)
beta=1;
M=size(obj);
for j=1:M(2)
    d=norm(obj(:,j)-h,1);
    
    Aux=r-d;
    
    Ficticia(j)=beta*(1/(1+exp(-70*(Aux+b))));
    
end

obstaculo=Ficticia;
end