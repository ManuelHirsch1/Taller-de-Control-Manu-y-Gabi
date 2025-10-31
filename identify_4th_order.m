% ====== USER CONFIG ======
infile = "log_escalon.csv";  % el que generaste en la adquisición
Ts     = 0.01;               % debe coincidir con el loop del Arduino (TS)

% ====== LOAD DATA ======
T = readtable(infile);
t  = T.t_s(:);
y  = T.theta_deg(:);     % salida
u  = T.phi_ref_deg(:);   % entrada

% (Opcional) recorta datos para quitar transitorio inicial sin escalón
% Ej: usa desde 0.5s hasta el final
idx = (t >= 0.5);
t = t(idx); y = y(idx); u = u(idx);

% ====== MAKE IDDATA ======
% Asegurate de que Ts es consistente con el paso de muestreo real:
% Ts_real = median(diff(t));  % si querés usar el paso real medido
data = iddata(y, u, Ts);

% (Opcional) Pre-procesado
% data = detrend(data);     % quita offset si lo necesitás
% data = idresamp(data, 1); % re-muestrear si hace falta

% ====== TF ESTIMATION (4º orden) ======
% Na: num order, Nb: den order
% Para 4º orden con ceros (elegí 2 o 3 ceros típico); probá distintos Na.
Na = 2;   % ceros
Nb = 4;   % polos (orden del denominador)
opt = tfestOptions('InitializeMethod','all','SearchMethod','auto');
sys_tf = tfest(data, Nb, Na, opt);

disp('--- Modelo TF estimado ---');
sys_tf = minreal(sys_tf) %#ok<NOPTS>
[p_tf, z_tf] = pzmap(sys_tf);
disp('Polos (TF):'); disp(p_tf);
disp('Ceros (TF):'); disp(z_tf);

% ====== SS ESTIMATION (4º orden) ======
% Modelo de 4 estados (orden 4)
opt_ss = ssestOptions('SearchMethod','auto');
sys_ss = ssest(data, 4, opt_ss);   % estado-espacio de orden 4

disp('--- Modelo SS estimado (orden 4) ---');
sys_ss = minreal(sys_ss) %#ok<NOPTS>
[p_ss, z_ss] = pzmap(sys_ss);
disp('Polos (SS):'); disp(p_ss);
disp('Ceros (SS):'); disp(z_ss);

% ====== VALIDATION PLOTS ======
figure; compare(data, sys_tf, sys_ss); grid on;
title('Comparación de modelos vs datos');

% ====== RESPUESTAS ======
figure; step(sys_tf); grid on; title('Step response - TF (identificado)');
figure; impulse(sys_tf); grid on; title('Impulse response - TF (identificado)');
