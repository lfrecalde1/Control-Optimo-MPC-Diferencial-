function Dinamica = COMPENSACION_DINAMICA_PLATAFORMA_MOVIL(vrefp,vref_e,v,q,ts,PARAMETROS)


% a) Velocidades de la plataforma móvil y del brazo robótico
     us = v(1);
     ws = v(2);

     
% b) Estados de la plataforma móvil y del brazo robótico
     ths = q(2);

% c) Parámetros Dinámicos de la plataforma movil

     C1=PARAMETROS(1);
     C2=PARAMETROS(2);
     C3=PARAMETROS(3);
     C4=PARAMETROS(4);
     C5=PARAMETROS(5);
     C6=PARAMETROS(6);
     C7=PARAMETROS(7);
% d) Matriz de Inercia
     M11 = C1;
     M12 = 0;
    
     M21 = 0;
     M22 = C2;
    
     M = [M11 M12;
          M21 M22];
 
% e) Matriz de Fuerzas Centrípetas y de Coriolis
     Cs11 = C3;
     Cs12 = C4+C5*ws;
    
     Cs21 =C6*ws;
     Cs22 =C7;
     
     C = [Cs11 Cs12;
          Cs21 Cs22];      
% f) MatrizCompensacion Dinamica.
K=[1 0
   0 1];
% h) Modelo para compensación dinámica 
    vref= M*(vrefp+K*vref_e)+C*v;
    us = vref(1);
    ws = vref(2);

    v_1 = [us ws]';
   
    Dinamica = [v_1];
end