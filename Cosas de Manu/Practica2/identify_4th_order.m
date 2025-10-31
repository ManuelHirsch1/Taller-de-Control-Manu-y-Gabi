% identify_4th_order.m  (versión corregida)

% ====== USER CONFIG ======
infile = "log_escalon.csv";
Ts     = 0.01;               % o usa Ts_real = median(diff(t));

% ====== LOAD DATA ======
T = readtable(infile);
t  = T.t_s(:);
y  = T.theta_deg(:);
u  = T.phi_ref_deg(:);

% (Opcional) recorte inicial
idx = (t >= 0.5);
t = t(idx); y = y(idx); u = u(idx);

% ====== IDDATA ======
% Si querés usar el paso real: Ts = median(diff(t));
data = iddata(y, u, Ts);

% ====== TF ESTIMATION (4º orden) ======
Na = 2;  % ceros (probá 1..3 si hace falta)
Nb = 4;  % polos (orden)
opt = tfestOptions('InitializeMethod','all','SearchMethod','auto');
sys_tf_id = tfest(data, Nb, Na, opt);   % <-- idtf (NO usar minreal directo)

% ====== SS ESTIMATION (orden 4) ======
opt_ss   = ssestOptions('SearchMethod','auto'); % 'DisturbanceModel','estimate' por defecto
sys_ss_id = ssest(data, 4, opt_ss);            % <-- idss

% ====== VALIDATION (usar modelos id* aquí) ======
figure; compare(data, sys_tf_id, sys_ss_id); grid on;
title('Comparación de modelos vs datos (id*)');

% ====== Conversiones para análisis (minreal, polos/ceros, step/impulse) ======
% --- TF: idtf -> tf ---
sys_tf = tf(sys_tf_id);            % LTI clásico
sys_tf = minreal(sys_tf);          % ahora sí
[p_tf, z_tf] = pzmap(sys_tf);

disp('--- Modelo TF (LTI) simplificado ---');
sys_tf
disp('Polos (TF):');  disp(p_tf);
disp('Ceros (TF):');  disp(z_tf);

% --- SS: idss -> ss (discreto con Ts) ---
sys_ss = ss(sys_ss_id);            % LTI clásico (discreto si Ts>0)
sys_ss = minreal(sys_ss);
[p_ss, z_ss] = pzmap(sys_ss);

%disp('--- Modelo SS (LTI) discreto simplificado ---');
sys_ss
disp('Polos (SS):');  disp(p_ss);
disp('Ceros (SS):');  disp(z_ss);

% (Opcional) continuo desde el discreto (si lo necesitás)
% sys_ss_c = d2c(sys_ss, 'zoh');   % comentar si no hace falta
% disp('SS continuo (opcional):'); sys_ss_c

% ====== RESPUESTAS ESTÁNDAR (sobre LTI) ======
figure; step(sys_tf); grid on; title('Step response - TF (LTI)');
figure; impulse(sys_tf); grid on; title('Impulse response - TF (LTI)');
