%% Convertir a espacio de estados la transferencia P obtenida en el TP2
% Planteo el primer lugar solo a theta como salida para verificar que
% funcione bien el control por realimentacion de estados.
close all;
format short g
s = tf('s');
P = -3.1122 * ((s+0.442) * (s^2 + 1.298*s +4.636)) / ...
    ((s^2 + 1.556*s + 2.591) * (s^2 + 1.138*s + 63.17));
[A,B,C,D] = ssdata(P); 

%% Punto de equilibrio
x1e=0;
x2e=0;
x3e=0;
x4e=0;
ue =0;
ye =x3e;

xeq = [x1e;x2e;x3e;x4e];

n = size(A,1);  % orden del sistema

%% Test de controlabilidad y observabilidad (continuo)

Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('Rango de la matriz de controlabilidad (cont): %d\n', rank(Co));
fprintf('Rango de la matriz de observabilidad  (cont): %d\n', rank(Ob));

if rank(Co) == size(A,1)
    disp('✅ La planta continua es completamente controlable.');
else
    disp('❌ La planta continua NO es completamente controlable.');
end

if rank(Ob) == size(A,1)
    disp('✅ La planta continua es completamente observable.');
else
    disp('❌ La planta continua NO es completamente observable.');
end

%% Sistema en espacio de estados discreto
Ts   = 20e-3;
sysd = c2d(ss(A,B,C,D), Ts);
Ad   = sysd.A;  Bd = sysd.B;  Cd = sysd.C; Dd = sysd.D;

Cod = ctrb(Ad, Bd);
Obd = obsv(Ad, Cd);

fprintf('Rango de la matriz de controlabilidad (disc): %d\n', rank(Cod));
fprintf('Rango de la matriz de observabilidad  (disc): %d\n', rank(Obd));

if rank(Cod) == size(Ad,1)
    disp('✅ La planta discreta es completamente controlable.');
else
    disp('❌ La planta discreta NO es completamente controlable.');
end

if rank(Obd) == size(Ad,1)
    disp('✅ La planta discreta es completamente observable.');
else
    disp('❌ La planta discreta NO es completamente observable.');
end

%% Diseño de controladores por realimentación de estados (SIN integral)
% Nuestros polos del sistema original eran: -114, -1.44, -0.5+7.5j,
% -0.5-7.5j
% Elegiste estos polos nuevos:
poles_ctrl =[-0.778+1.41i -0.778-1.41i -2.72+7.47i -2.72-7.47i];

% Digitalizo los polos mediante polyd = exp(poles * Ts)
polesd_ctrl = exp(poles_ctrl * Ts);

% Ganancias de realimentación de estados
K  = place(A ,B ,poles_ctrl);
Kd = place(Ad,Bd,polesd_ctrl);

%% Observador de Luenberger (mismos polos que elegiste)
poles_obs  = [-4 -5 -25 -26];
polesd_obs = exp(poles_obs * Ts);

L  = place(A' , C', poles_obs )';
Ld = place(Ad', Cd',polesd_obs)';

%% Chequeo la estabilidad del sistema discreto realimentado con observador (SIN integral)
Adcl = Ad - Bd*Kd;        % dinámica cerrada del control
Eobs = Ad - Ld*Cd;        % dinámica del error de estimación

rho_ctrl = max(abs(eig(Adcl)));
rho_obs  = max(abs(eig(Eobs)));

fprintf('max|lambda(Ad-Bd*Kd)| = %.4f\n', rho_ctrl);
fprintf('max|lambda(Ad-Ld*Cd)| = %.4f\n', rho_obs);

if rho_ctrl < 1 && rho_obs < 1
    disp('✅ Estable con observador (por separación, sin integral).');
else
    disp('❌ Inestable (sin integral): revisá polos de control u observador.');
end

% Muestro resultados
disp('=== Ganancia de control continuo ==='); disp(K);
disp('=== Ganancia de control discreto ==='); disp(Kd);
fprintf('\n||K||=%.3e, ||Kd||=%.3e\n', norm(K), norm(Kd));
disp('=== Observador de Luenberg continuo ==='); disp(L);
disp('=== Observador de Luenberg discreto ==='); disp(Ld);
fprintf('\n||L||=%.3e, ||Ld||=%.3e\n', norm(L), norm(Ld));

%% ================================
%      ACCIÓN INTEGRAL (DISCRETO)
%  Integración del error en theta
% ================================

% Integral del error e(k) = r(k) - y(k); en la dinámica homogénea (r=0):
% xI(k+1) = xI(k) - Ts * y(k)  = xI(k) - Ts * Cd * x(k)
% Estado aumentado: xa = [x; xI]

Ai = [Ad          zeros(n,1);
     -Cd*Ts      1       ];   % 5x5
Bi = [Bd;
      0];                     % 5x1

% Test de controlabilidad del sistema aumentado
CoAi = ctrb(Ai, Bi);
fprintf('\nRango controlabilidad sistema aumentado (con integral): %d\n', rank(CoAi));
if rank(CoAi) == size(Ai,1)
    disp('✅ El sistema aumentado es completamente controlable.');
else
    disp('❌ El sistema aumentado NO es completamente controlable.');
end

%% Polos del controlador aumentado (USANDO TUS 4 POLOS + 1 NUEVO PARA EL INTEGRAL)
% Usamos exactamente tus polos discretos para los 4 estados originales...
% y agregamos un polo extra para el integrador, más lento que los otros.
p_int_cont = -0.025;                 % polo continuo extra
p_int_disc = exp(p_int_cont*Ts); % versión discreta

polesd_ctrl_aug = [polesd_ctrl  p_int_disc];  % 5 polos en total

% Lugar de polos para el sistema aumentado
Kdi = place(Ai, Bi, polesd_ctrl_aug);

% Separo parte de estados y parte integral
Kd_int = Kdi(1:n);   % realimentación de estados (4 estados)
Ki     = Kdi(end);   % ganancia integral

disp('=== Ganancia de control discreta con acción integral ===');
disp('Kd_int ='); disp(Kd_int);
disp('Ki     ='); disp(Ki);

%% Chequeo estabilidad del lazo cerrado aumentado (con integral)
Ad_aug_cl = Ai - Bi*Kdi;
rho_aug = max(abs(eig(Ad_aug_cl)));
fprintf('max|lambda(Ai-Bi*Kdi)| = %.4f\n', rho_aug);

if rho_aug < 1
    disp('✅ Estable con observador + acción integral.');
else
    disp('❌ Inestable con integral: habría que reubicar el polo del integrador.');
end
