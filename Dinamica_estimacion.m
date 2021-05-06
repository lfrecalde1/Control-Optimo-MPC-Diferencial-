function f=Dinamica_estimacion(z1,uc,wc,u,w,ue,we,H,N,k,ts,x)


x=z1;
F=[];

va(1,1)=ue(1,k-N);

va(2,1)=we(1,k-N);

uref_c(1)=uc(1,k-N);

wref_c(1)=wc(1,k-N);

va_r(1,1)=u(1,k-N);

va_r(2,1)=w(1,k-N);

h=[va(1,1);va(2,1)];

hd=[va_r(1);va_r(2)];

% L = [Fun(H,hd,h)];

for i=1:N
    
    vref=[uref_c(i);wref_c(i)];
 % d) Matriz de Inercia
     M11 = x(1);
     M12 = 0;
    
     M21 = 0;
     M22 = x(2);
    
     M = [M11 M12;
          M21 M22];
 
% e) Matriz de Fuerzas Centrípetas y de Coriolis
     Cs11 = x(3);
     Cs12 = x(4)+x(5)*va(2,i);
    
     Cs21 =x(6)*va(2,i);
     Cs22 =x(7);
     
     C = [Cs11 Cs12;
          Cs21 Cs22];

    %Dinamica
   vp = pinv(M)*(vref-C*va(:,i));
   
   va(:,i+1)=vp*ts+va(:,i);
   
   h=va(:,i+1);
   
   
   hd=[u(1,k-(N-i));w(1,k-(N-i))];
   
   L(i) = Fun(H,hd,h);
   
   uref_c(i+1)=uc(1,k-(N-i));
   
   wref_c(i+1)=wc(1,k-(N-i));
    
end
  %f=sqrt(F'*F); % funcion de minimizacion
 f=0.5*sum(L); % funcion de minimizacion
% F'
end
function [F] = Fun(H,hd,h)
    alpha=0.1;    %Peso de errores de posici�n
    he = hd-h;    %Errores de control
    F = alpha*he'*H*he;  
end