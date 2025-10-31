%% TP2 - Control de la planta del péndulo
close all;
s=tf('s');
% Configuración del Bode
optionss=bodeoptions;
optionss.MagVisible='on';
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=-180;
optionss.PhaseMatchingFreq=1;
optionss.Grid='on';

%% Planta
s = tf('s');
%P = sys_est;
P = (-107.2*s^3-157.7*s^2-1413*s-107.5)/(s^4+117.6*s^3+494.9*s^2+1.296e4*s+1.808e4);

% Mostrar la función de transferencia
disp('Transferencia estimada:');
P

figure;
bode(P,optionss);
title("Diagrama de Bode de P");


%% Control Proporcional
C_prop=zpk([],[],1);
Ts=20e-3;

C_prop_dig=c2d(C_prop,Ts,'tustin');

figure;
rlocus(P*C_prop);
title("Root Locus de lazo usando un controlador proporcional");

% Para ganancias de hasta 0.7 el sistema es estable. Pruebo en simulink.


%% Control PD

