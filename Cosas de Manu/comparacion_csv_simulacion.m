%% === Script simple: CSV vs impulso de T(s) ===
clear; clc; close all;

% === 1) Archivo CSV ===
csv_path = "registro_control_P.csv";  % cambiá el nombre si hace falta
M = readmatrix(csv_path);   % espera 2 columnas: [tiempo, señal]
t_csv = M(:,1);
y_csv = M(:,2);

% Opcional: alinear tiempo para que 1.22 s -> 0 s
t_csv = t_csv - 1.22;

% === 2) Planta P(s) ===
s = tf('s');
P = (-90*s^2-720*s) / ...
    (s^4+116.7*s^3+337.6*s^2+6702*s+9346);

C_p = -0.7;
S = feedback(1, P*C_p);   % 1 / (1 + P*C)
[y_imp, t_imp] = impulse(S);

% === 6) Gráfico (cada señal con su propio tiempo) ===
figure; hold on; grid on; box on;
plot(t_imp, -40*y_imp, 'LineWidth', 1.8);                 % Impulso de T(s)
plot(t_csv, y_csv, '--', 'LineWidth', 1.4);           % Señal del CSV
xlabel('Tiempo [s]'); ylabel('Amplitud');
title('Control proporcional');
legend('Impulso de S(s) simulado', 'Comportamiento real', 'Location', 'best');

%%
% === 1) Archivo CSV ===
csv_path_pd = "registro_control_PD.csv";  % cambiá el nombre si hace falta
M2 = readmatrix(csv_path_pd);   % espera 2 columnas: [tiempo, señal]
t_csv2 = M2(:,1);
y_csv2 = M2(:,2);

% Opcional: alinear tiempo para que 1.22 s -> 0 s
t_csv2 = t_csv2 - 0.5;

% === 2) Planta P(s) ===
s = tf('s');
P = (-90*s^2-720*s) / ...
    (s^4+116.7*s^3+337.6*s^2+6702*s+9346);


% Controlador pd
C_pd = -0.7-0.03*s;
S2 = feedback(1, P*C_pd);   % 1 / (1 + P*C)
[y_imp2, t_imp2] = impulse(S2);

% === 6) Gráfico (cada señal con su propio tiempo) ===
figure; hold on; grid on; box on;
plot(t_imp2, 30*y_imp2, 'LineWidth', 1.8);                 % Impulso de T(s)
plot(t_csv2, y_csv2, '--', 'LineWidth', 1.4);           % Señal del CSV
xlabel('Tiempo [s]'); ylabel('Amplitud');
title('Control PD');
legend('Impulso de S(s)', 'Señal real', 'Location', 'best');
%%
% === 1) Archivo CSV ===
csv_path_pi = "registro_control_PI.csv";  % cambiá el nombre si hace falta
M3 = readmatrix(csv_path_pi);   % espera 2 columnas: [tiempo, señal]
t_csv3 = M3(:,1);
y_csv3 = M3(:,2);

% Opcional: alinear tiempo para que 1.22 s -> 0 s
t_csv3 = t_csv3 - 1.08;

% === 2) Planta P(s) ===
s = tf('s');
P = (-90*s^2-720*s) / ...
    (s^4+116.7*s^3+337.6*s^2+6702*s+9346);


% Controlador pd
C_pi = -0.7-0.3/s;
S3 = feedback(1, P*C_pi);   % 1 / (1 + P*C)
[y_imp3, t_imp3] = impulse(S3);

% === 6) Gráfico (cada señal con su propio tiempo) ===
figure; hold on; grid on; box on;
plot(t_imp3, -30*y_imp3, 'LineWidth', 1.8);                 % Impulso de T(s)
plot(t_csv3, y_csv3, '--', 'LineWidth', 1.4);           % Señal del CSV
xlabel('Tiempo [s]'); ylabel('Amplitud');
title('Control PI');
legend('Impulso de S(s)', 'Señal real', 'Location', 'best');

% Configuración del Bode
my_bode_options = bodeoptions;
my_bode_options.PhaseMatching = 'on';
my_bode_options.PhaseMatchingFreq = 1;
my_bode_options.PhaseMatchingValue = -180;
my_bode_options.Grid = 'on';
%my_bode_options.XLim = {[1 100]};

figure;
bode(P, my_bode_options);