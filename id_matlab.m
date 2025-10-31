%% === 1. Configuración del puerto serie ===
close all;
% Cambiá el COM según tu PC (ejemplo COM3 en Windows, /dev/ttyUSB0 en Linux)
arduinoPort = "COM3";
baudRate    = 115200;

s = serialport(arduinoPort, baudRate);

% Esperar a que arranque Arduino y limpie el buffer
flush(s);
pause(2);

% Arduino imprime: t_s,phi_ref_deg,theta_imu_deg
% Definimos cuántos segundos queremos grabar
T_total = 15;       % duración de la prueba en segundos
Ts      = 0.01;     % periodo de muestreo (coincide con delay(10) en Arduino)
N       = round(T_total / Ts);

t      = zeros(N,1);
u      = zeros(N,1);
y      = zeros(N,1);

%% === 2. Leer datos del Arduino ===
disp("Grabando datos desde Arduino...");
for k = 1:N
    line = readline(s);
    vals = str2double(split(line, ","));
    if numel(vals) == 3
        t(k) = vals(1);
        u(k) = vals(2);   % entrada
        y(k) = vals(3);   % salida
    end
end
disp("Grabación finalizada.");

clear s; % cerrar puerto serie

%% === 2.1 Recortar datos desde t >= 5 s (SOLO este cambio) ===
t0_keep = 5.0;
idx = t >= t0_keep;               % máscara de tiempo
t = t(idx) - t(find(idx,1,'first'));  % re-referenciar tiempo a 0 en el segmento
u = u(idx);
y = y(idx);

%% === 3. Construir objeto de identificación ===
% (Se mantiene tu Ts fijo)
data = iddata(y, u, Ts);

%% === 4. Ajustar función de transferencia (4 polos, 3 ceros) ===
nz = 3;  % número de ceros
np = 4;  % número de polos
sys_est = tfest(data, np, nz);

%% === 5. Validar modelo ===
figure;
compare(data, sys_est);

% Mostrar polos y ceros
disp("Polos estimados:");
disp(pole(sys_est));
disp("Ceros estimados:");
disp(zero(sys_est));

% Respuesta al escalón estimada
figure;
step(sys_est);
title("Respuesta al escalón del modelo identificado");

figure;
impulse(sys_est);
