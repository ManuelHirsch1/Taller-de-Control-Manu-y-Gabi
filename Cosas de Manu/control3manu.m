%% ============================================
%  CONTROL POR REALIMENTACIÓN DE ESTADOS (SIN INTEGRAL)
%  Para implementación en Arduino
%  - Planta: u -> θ (en grados)
%  - Estados físicos: [φ; φ_dot; θ; θ_dot]
%  - Control: u = -K * x_hat
%  - Observador de Luenberger: estima x_hat a partir de θ
% ============================================

clear; clc;

%% --- 1. Definición de la planta continua (u -> theta)
s = tf('s');
P = -90 * (s*(s+8)) / ((s+114)*(s+1.44)*(s^2 + s + 56.5));
P = minreal(P);
[num, den] = tfdata(P,'v');

% Realización en espacio de estados (forma canónica controlable)
[A0, B0, C0, D0] = tf2ss(num, den);
n = size(A0,1);

%% --- 2. Cambio de base a estados físicos [phi; phi_dot; theta; theta_dot]
% Queremos que los estados representen magnitudes físicas interpretables
% Definimos:
%   x3 = theta  = C0*x0
%   x4 = theta_dot = C0*A0*x0  (derivada de salida)
t3 = C0;
t4 = C0*A0;

% Completamos T con filas independientes (para phi y phi_dot)
E = eye(n);
rows = [];
for i = 1:n
    cand = E(i,:);
    if rank([t3; t4; rows; cand],1e-10) > rank([t3; t4; rows],1e-10)
        rows = [rows; cand];
    end
    if size(rows,1)==2, break; end
end

T = [rows; t3; t4];      % matriz de cambio de base
V = inv(T);

A = T*A0*V;
B = T*B0;
C = C0*V;
D = D0;

sys = ss(A,B,C,D);
sys.StateName = {'phi','phi_dot','theta','theta_dot'};

disp('Chequeo de la nueva base:')
disp('C (debería ser [0 0 1 0]):'); disp(C);
disp('Fila 3 de A (debería ser [0 0 0 1]):'); disp(A(3,:));
disp('B(3) (debería ser ~0):'); disp(B(3));

fprintf('\nTransferencia original vs nueva:\n');
[minreal(tf(sys)), P];

%% --- 3. Diseño del controlador LQR (sin acción integral)
Q = C.'*C + 1e-4*eye(n);  % penaliza principalmente salida theta
R = 100;                  % penaliza esfuerzo de control

[K,~,~] = lqr(A,B,Q,R);

fprintf('\nGanancia del controlador (base física):\n');
disp('K = [k_phi, k_phi_dot, k_theta, k_theta_dot]');
disp(K);

%% --- 4. Diseño del observador de Luenberger
% Calcula los polos del sistema regulado
p_reg = eig(A - B*K);
factor_rapidez = 4;       % observador 4x más rápido
p_obs = p_reg * factor_rapidez;
L = place(A.', C.', p_obs).';

fprintf('\nGanancia del observador (L):\n');
disp(L);

%% --- 5. Discretización para implementación en Arduino
Ts = 0.02; % 20 ms

sysc = ss(A,B,C,D);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

% L y K son continuos, pero para Ts pequeño sirven casi igual
% Aun así, discretizamos el observador explícitamente
Ld = (eye(n) + A*Ts) * L; % aproximación de Euler

fprintf('\nMATRICES DISCRETAS (para Arduino)\n');
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);
disp('Cd ='); disp(Cd);
disp('Ld ='); disp(Ld);
disp('K  ='); disp(K);

%% --- 6. Exportar datos listos para pegar en Arduino
fprintf('\n--- COPIAR ESTAS MATRICES A ARDUINO ---\n');
fprintf('float Ad[4][4] = {\n');
for i=1:4
    fprintf('  {%.6f, %.6f, %.6f, %.6f}', Ad(i,:));
    if i<4, fprintf(',\n'); else, fprintf('\n'); end
end
fprintf('};\n');

fprintf('float Bd[4] = {%.6f, %.6f, %.6f, %.6f};\n', Bd);
fprintf('float Cd[4] = {%.6f, %.6f, %.6f, %.6f};\n', Cd);
fprintf('float Ld[4] = {%.6f, %.6f, %.6f, %.6f};\n', Ld);
fprintf('float K[4]  = {%.6f, %.6f, %.6f, %.6f};\n', K);



