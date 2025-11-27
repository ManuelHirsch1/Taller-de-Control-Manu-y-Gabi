%% Convertir a espacio de estados la transferencia P obtenida en el TP2
% Planteo el primer lugar solo a theta como salida para verificar que
% funcione bien el control por realimentacion de estados.
close all;
format short g
s = tf('s');
P = -3.1122 * ((s+0.442) * (s^2 + 1.298*s +4.636)) / ((s^2 + 1.556*s + 2.591) * (s^2 + 1.138*s + 63.17));
[A,B,C,D] = ssdata(P); 


%% Control por realimentación de estados con observador
x1e=0;
x2e=0;
x3e=0;
x4e=0;
ue=0;
ye=x3e;

xeq = [x1e;x2e;x3e;x4e];
s = tf('s');

n = size(A,1);  % orden del sistema

% Test de controlabilidad y observabilidad

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

%% Sistema en espacio de estados discreto
% Discretizamos el sistema
Ts   = 20e-3;
sysd = c2d(ss(A,B,C,D), Ts);
Ad   = sysd.A;  Bd = sysd.B;  Cd = sysd.C; Dd = sysd.D;
% Test de controlabilidad y observabilidad

Cod = ctrb(Ad, Bd);
Obd = obsv(Ad, Cd);

fprintf('Rango de la matriz de controlabilidad: %d\n', rank(Cod));
fprintf('Rango de la matriz de observabilidad:  %d\n', rank(Obd));

if rank(Cod) == size(Ad,1)
    disp('✅ La planta es completamente controlable.');
else
    disp('❌ La planta NO es completamente controlable.');
end

if rank(Obd) == size(Ad,1)
    disp('✅ La planta es completamente observable.');
else
    disp('❌ La planta NO es completamente observable.');
end

%% Diseño de controladores por realimentación de estados
% Se diseñan los controladores mediante la colocación de los polos
% Nuestros polos del sistema original eran: -114, -1.44, -0.5+7.5j,
% -0.5-7.5j
% Queremos buscar otros que mejoren un poco el desempeño del sistema
% mediante su realimentacion por estados.

poles_ctrl =[-0.778+1.41i -0.778-1.41i -2.72+7.47i -2.72-7.47i];
% Digitalizo los polos mediante una transformación al plano complejo a partir
% de polyd = exp(poles * Ts)
polesd_ctrl = exp(poles_ctrl * Ts);

% Usando place planteo el vector de ganancia de realimentación de estados
K = place(A,B,poles_ctrl);
Kd = place(Ad,Bd,polesd_ctrl);

%% Planteo los polos del observador del doble de rápidos que los del controlador
poles_obs  = [-4 -5 -25 -26];

polesd_obs = exp(poles_obs * Ts);

L = place(A', C', poles_obs)';
Ld = place(Ad', Cd',polesd_obs).';

%% Chequeo la estabilidad del sistema discreto realimentado con observador
Adcl = Ad - Bd*Kd;        % dinámica cerrada del control
Eobs = Ad - Ld*Cd;        % dinámica del error de estimación

rho_ctrl = max(abs(eig(Adcl)));
rho_obs  = max(abs(eig(Eobs)));

fprintf('max|lambda(Ad-Bd*Kd)| = %.4f\n', rho_ctrl);
fprintf('max|lambda(Ad-Ld*Cd)| = %.4f\n', rho_obs);

if rho_ctrl < 1 && rho_obs < 1
    disp('✅ Estable con observador (por separación).');
else
    disp('❌ Inestable: revisá polos de control u observador.');
end

% Muestro resultados
disp('=== Ganancia de control continuo ==='); disp(K);
disp('=== Ganancia de control discreto ==='); disp(Kd);
fprintf('\n||K||=%.3e, ||Kd||=%.3e\n', norm(K), norm(Kd));
disp('=== Observador de Luenberg continuo ==='); disp(L);
disp('=== Observador de Luenberg discreto ==='); disp(Ld);
fprintf('\n||L||=%.3e, ||Ld||=%.3e\n', norm(L), norm(Ld));

