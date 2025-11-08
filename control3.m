%% Convertir a espacio de estados la transferencia P obtenida en el TP2
% Planteo el primer lugar solo a theta como salida para verificar que
% funcione bien el control por realimentacion de estados.
s = tf('s');
P = -90 * (s * (s + 8)) / ((s + 114) * (s + 1.44) * (s^2 + s + 56.5));

% Extraigo numerador y denominador como vectores (coeficientes en
% potencias descendentes de s)
[num, den] = tfdata(P, 'v');

% Realización en espacio de estados (forma canónica controlable)
[A0, B0, C0, D0] = tf2ss(num, den);   % orden 4, D0 = 0

% garantiza controlabilidad. La última fila de A0 contiene (con signo) los
% coeficientes del denominador; las columnas superiores generan la cadena
% de integradores.

% Queremos definir nuevos estados:
% x3 = theta = y = C0 * x0
% x4 = theta_dot = (C0*A0) * x0   (derivada de la salida, para u=0)

t3 = C0;        % fila 3 de T
t4 = t3*A0;     % fila 4 de T

% Completamos T con dos filas adicionales independientes (para phi y phi_dot).
% Usamos filas de la base canónica que aumenten el rango.
E = eye(4);
rows = [];
for i = 1:4
    cand = E(i,:);
    if rank([t3; t4; rows; cand], 1e-10) > rank([t3; t4; rows], 1e-10)
        rows = [rows; cand];
    end
    if size(rows,1) == 2, break; end
end

% Orden de estados nuevo: [phi; phi_dot; theta; theta_dot]
T = [rows; t3; t4];      % 4x4 e invertible por construcción
V = inv(T);

A = T*A0*V;
B = T*B0;
C = C0*V;
D = D0;

% Nombramos estados
sys = ss(A,B,C,D);
sys.StateName = {'phi','phi_dot','theta','theta_dot'};

%% Chequeos rápidos
disp('C (debería ser [0 0 1 0]):'); disp(C);
disp('Fila 3 de A (debería ser [0 0 0 1]):'); disp(A(3,:));
disp('B(3) (idealmente 0 por grado relativo 2):'); disp(B(3));

% Verificar que mantiene la misma transferencia
fprintf('Comparación de la TF original y la obtenida:\n');
minreal(tf(sys)), P

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
%% Pruebo primero sin acción integral
% Nuestros polos del sistema original eran: -114, -1.44, -0.5+7.5j,
% -0.5-7.5j
% Queremos buscar otros que mejoren un poco el desempeño del sistema
% mediante su realimentacion por estados.

% Calculo de K por LQR (continuo)
% Heurística simple: penalizar salida (C'*C) + pequeña identidad; esfuerzo R=1
Q = C.'*C +1e-4*eye(size(A)); % Subir polos mas a la iqz
R = 100; %polos mas cerca de im
[Ko,~,~] = lqr(A,B,Q,R);          % u = -Ko x

% Calculo L por colocación de polos (observador más rápido)
p_reg = eig(A - B*Ko);            % polos del regulador
factor_rapidez = 4;              % 3–5 suele andar bien
p_obs = p_reg * factor_rapidez;  % más a la izquierda (más rápidos)
Lo = place(A.', C.', p_obs).';    % ganancia del observador

Ts = 0.02; % 20 ms

sysc = ss(A,B,C,0);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

Ko_d = Ko;  % tus ganancias sirven directamente (no cambian mucho en Ts pequeño)
Lo_d = Lo;  % igual, discretizaremos con Ts multiplicado

% Para observador discreto más correcto:
Lo_d = (eye(size(A)) + A*Ts) * Lo;  % aproximación de Euler

format long
Ad, Bd, Cd, Ko_d, Lo_d

%% === Control con acción integral (versión LQI robusta con fuga) ===
% Usamos la misma planta P -> theta, y matrices A,B,C,D ya definidas

% --- Definición del integrador con fuga (para evitar inestabilidad)
a = 0.5;   % fuga pequeña (0.1–1 típicamente). Evita acumulación infinita si hay bias.

Aaug = [A,             zeros(n,1);
        -C,            -a        ];
Baug = [B;
        0];

% Matrices de pesos para el LQR (LQI = LQR sobre sistema aumentado)
Qx = C.'*C + 1e-2*eye(n);   % penaliza principalmente el error de salida y levemente los estados
Qz = 50;                    % penaliza el error integral (cuanto más grande, menor offset)
Q  = blkdiag(Qx, Qz);
R  = 1;                     % penalización sobre la señal de control u

% --- Cálculo de ganancia óptima
[Kaug,~,~] = lqr(Aaug, Baug, Q, R);
K  = Kaug(1:n);     % Ganancia de estados
Ki = Kaug(end);     % Ganancia del integrador

% --- Observador de Luenberger (para los estados x, no para z)
p_reg = eig(A - B*K);           % polos del regulador
factor_rapidez = 4;             % observador 4x más rápido que el regulador
p_obs = p_reg * factor_rapidez; % desplazamiento a la izquierda

L = place(A.', C.', p_obs).';   % ganancia del observador

%% --- Resultados
disp('Ganancias del controlador:');
disp('K (estados):'); disp(K);
disp('Ki (integral):'); disp(Ki);
disp('Ganancia del observador L:'); disp(L);
disp('Factor de fuga a:'); disp(a);

