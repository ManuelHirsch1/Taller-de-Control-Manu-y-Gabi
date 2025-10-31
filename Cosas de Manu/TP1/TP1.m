% TP1, Bola Suspendida
clear all;close all;clc
s=tf('s');
% Configuración del Bode
my_bode_options = bodeoptions;
my_bode_options.PhaseMatching = 'on';
my_bode_options.PhaseMatchingFreq = 1;
my_bode_options.PhaseMatchingValue = -180;
my_bode_options.Grid = 'on';
%my_bode_options.XLim = {[1 100]};

%Valores de linealización
%Valores de equilibrio
x1e=1;
x2e=1;
x3e=0;
ue=4;
ye=x2e;

%Parámetros del problema
R=4;
L=1e-2;
m=0.1;
g=10;

% Definir variables como simbolicas
syms x1 x2 x3 u y;
x = [x1; x2; x3];

% Definir funciones de derivadas (f)
f1 = u/L-R/L*x1;    %Dinamica del primer tanque cónico
f2 = x3;  %Dinámica del segundo tanque cónico
f3 = g-x1/(x2*m);  %Dinámica del actuador
f = [f1; f2; f3];

% Definir la salida y
y = x2;

% Defino los jacobianos
A = jacobian(f,x);
B = jacobian(f,u);
C = jacobian(y,x);
D = jacobian(y,u);

% Evaluar el jacobiano en los valores de equilibrio
A = subs(A, str2sym({'x1', 'x2', 'x3', 'u', 'y'}), {x1e, x2e, x3e, ue, ye});
B = subs(B, str2sym({'x1', 'x2', 'x3', 'u', 'y'}), {x1e, x2e, x3e, ue, ye});
C = subs(C, str2sym({'x1', 'x2', 'x3', 'u', 'y'}), {x1e, x2e, x3e, ue, ye});
D = subs(D, str2sym({'x1', 'x2', 'x3', 'u', 'y'}), {x1e, x2e, x3e, ue, ye});

Ass = double(A);
Bss = double(B);
Css = double(C);
Dss = double(D);

% Transferencia Linealizada (Planta)
P = zpk(ss(Ass, Bss, Css, Dss));

%%
% Se diseña un controlador.
C1 = zpk([-3.162 -3.162 -400], [0 -4000 -4000], -1);

figure;
bode(P*C1, my_bode_options);
title("Bode de lazo sin ajustar ganancia de controlador")

% Definimos Wgc para que en ese punto haya 65 de MF
wgc = 850;
k = db2mag(143);
C1g = C1*k;

figure;
bode(C1g*P, my_bode_options);
title("Bode de lazo ajustando ganancia del controlador")

% POnemos la digitalización
Ts = 4*tand(2.5)/wgc;
Dig = zpk(4/Ts, -4/Ts, -1);

figure;
bode(C1g*P*Dig, my_bode_options);
title("Bode agregando digitalización")
% Se ve que tenemos margen de fase 60°.

%%
L = C1g*P*Dig;
T = feedback(L, 1);
figure;
step(T);
grid on;

% Se obtuvo un sobre pico menor al 10% y un settling time mucho menor a 1s.
% la dinámica pasatodo del muestreo impone un techo en la wgc.
% se ve que el controlador cumple con las especificaciones de diseño
% solicitadas en la consigna.

% margen de estabilidad
S = 1/(1+L);
figure;
bode(S, my_bode_options);

sm = 1/db2mag(3.35);
disp(['El margen de estabilidad es ', num2str(sm)]);

%% 1. Planta original
xeq = [x1e;x2e;x3e];
s = tf('s');

A = Ass;
B = Bss;
C = Css;
D = Dss;

n = size(A,1);  % orden del sistema

% 1b. Test de controlabilidad y observabilidad

Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('Rango de la matriz de controlabilidad: %d\n', rank(Co));
fprintf('Rango de la matriz de observabilidad:  %d\n', rank(Ob));

if rank(Co) == size(A,1)
    disp('✅ La planta es completamente controlable.');
else
    disp('❌ La planta NO es completamente controlable.');
end

if rank(Ob) == size(A,1)
    disp('✅ La planta es completamente observable.');
else
    disp('❌ La planta NO es completamente observable.');
end


% 2. Sistema aumentado (para acción integral)
% z_dot = r - y = r - Cx
Aa=[A zeros(n,1);-C 0];   % Tener especial cuidado en como esta definido en el simulink, deben coincidir ambas para ver el signo de C
Ba=[B;-D];  % El integrador ya no recibe -D

% 3. Controlador con acción integral
poles_aug = [-400 -4 -4 -4];  % Polos deseados para sistema aumentado
Ka = acker(Aa, Ba, poles_aug);
Kest  = Ka(1:n);     % Ganancia de estados
kI = -Ka(end);     % Ganancia del integrador (positivo)

% 4. Ganancia de referencia (para error en régimen = 0)
% kr = 1 / ( C * inv(A - B*Kest) * B );

% 5. Observador de estados
Lo = acker(A', C', [-800 -8 -8])';  % Polos del observador
