%% ============================================================
%  Diseño de control por estados + observador
%  Partiendo de una planta CONTINUA G(s)
%  y discretizando para Arduino con Ts.
% ============================================================

clear; clc; close all;

%% 1) Definir planta continua G(s) manualmente

% Ejemplo ficticio, reemplazá numc y denc por los que anotaste:
% G(s) = (b0 s^m + ... ) / (s^n + ...)

numc = [-3.1122  -5.4167   -16.2160  -6.3779];                 % <-- tus coeficientes del numerador
denc = [1.0000  2.6935   67.5269  101.2144  163.6741];      % <-- tus coeficientes del denominador

sysc = tf(numc, denc);             % planta continua
disp('Planta continua G(s):');
sysc

%% 2) Definir período de muestreo y discretizar

Ts = 0.020;                        % <-- tu Ts para Arduino (20 ms por ej.)

sysd = c2d(sysc, Ts, 'zoh');       % discretización por ZOH
disp('Planta discreta Gd(z):');
sysd

[Ad, Bd, Cd, Dd] = ssdata(ss(sysd));   % espacio de estados discreto mínimo

n = size(Ad,1);
m = size(Bd,2);
p = size(Cd,1);

fprintf('Orden del sistema: n=%d, entradas m=%d, salidas p=%d\n', n, m, p);

%% 3) Controlabilidad y observabilidad

Co = ctrb(Ad, Bd);
Ob = obsv(Ad, Cd);

rCo = rank(Co);
rOb = rank(Ob);

fprintf('Rango de controlabilidad: %d\n', rCo);
fprintf('Rango de observabilidad: %d\n', rOb);

if rCo < n
    warning('Sistema NO completamente controlable.');
end
if rOb < n
    warning('Sistema NO completamente observable.');
end

%% 4) Elegir polos del CONTROLADOR
% Opción A: elegir polos directamente en DISCRETO (plano z) -> usar p_cl_d
% Opción B: elegir polos en CONTINUO (plano s) -> usar p_cl_s y mapear a z

% ==== OPCION B (lo que preguntaste) ====
% Ejemplo de polos deseados en continuo (reemplazá por los tuyos):
% p_cl_s = [ -5   -6   -7+5j   -7-5j ];

% ---> COMPLETAR <---
p_cl_s = [-0.778+1.41i -0.778-1.41i -1.38+7.82i -1.38-7.82i];    % polos deseados en continuo (s)

% Mapear a discreto
p_cl_d = exp(p_cl_s * Ts);

disp('Polos deseados del controlador en continuo:');
disp(p_cl_s);
disp('Polos deseados del controlador en discreto (z):');
disp(p_cl_d);

if numel(p_cl_d) ~= n
    error('La cantidad de polos deseados debe ser %d (orden del sistema).', n);
end

K = place(Ad, Bd, p_cl_d);

disp('---------------------------------------------');
disp('Ganancia de realimentación de estados K (discreta):');
disp(K);

Acl = Ad - Bd*K;
disp('Polos de lazo cerrado (discreto):');
disp(eig(Acl));

%% 5) Elegir polos del OBSERVADOR

% Igual idea:
% p_obs_s = [ ... ];   % polos deseados del observador en continuo

% ---> COMPLETAR <---
p_obs_s = [-1.8 -1.82 -10 -10.2];    % polos observador en continuo (s)

p_obs_d = exp(p_obs_s * Ts);

disp('Polos deseados del observador en continuo:');
disp(p_obs_s);
disp('Polos deseados del observador en discreto (z):');
disp(p_obs_d);

if numel(p_obs_d) ~= n
    error('La cantidad de polos deseados para el observador debe ser %d.', n);
end

L = place(Ad', Cd', p_obs_d).';    % L = (place(Ad',Cd',p_obs_d))'

disp('---------------------------------------------');
disp('Ganancia del observador L (discreta):');
disp(L);

Aobs = Ad - L*Cd;
disp('Polos de la dinámica del error del observador (discreto):');
disp(eig(Aobs));

%% 6) Resumen para Arduino

disp('============================================');
fprintf('Ts = %.6f s\n', Ts);
disp('Ad = '); disp(Ad);
disp('Bd = '); disp(Bd);
disp('Cd = '); disp(Cd);
disp('Dd = '); disp(Dd);
disp('K = ');  disp(K);
disp('L = ');  disp(L);

fprintf('\n// --- Ganancia K para Arduino (fila) ---\n');
fprintf('float K[1][%d] = { {', n);
for i = 1:n
    if i < n
        fprintf('%.8f, ', K(i));
    else
        fprintf('%.8f', K(i));
    end
end
fprintf('} };\n');

fprintf('\n// --- Ganancia L para Arduino (columna) ---\n');
fprintf('float L[%d][1] = {\n', n);
for i = 1:n
    if i < n
        fprintf('  {%.8f},\n', L(i));
    else
        fprintf('  {%.8f}\n', L(i));
    end
end
fprintf('};\n');
