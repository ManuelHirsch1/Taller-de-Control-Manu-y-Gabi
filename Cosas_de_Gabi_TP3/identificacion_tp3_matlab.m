%% Identificación de planta péndulo-servo desde datos Arduino
% Asegurate de:
%  1) Haber subido el sketch de escalones al Arduino.
%  2) Conocer el puerto serie (por ej. "COM3" en Windows, "/dev/ttyACM0" en Linux).
%  3) Tener System Identification Toolbox para usar tfest.

clear; clc; close all;

%% Parámetros de conexión serie
puerto = "COM9";   % <-- CAMBIÁ ESTO SEGÚN TU PC
baud   = 115200;

% Crear objeto de puerto serie (MATLAB R2019b+ usa serialport)
s = serialport(puerto, baud);
configureTerminator(s, "LF");
flush(s);  % limpia buffer

disp('Esperando datos de Arduino...');

% Leer la cabecera CSV
header = strtrim(readline(s));
disp("Cabecera recibida: " + header);

%% Leer datos hasta recibir "END"
t = [];
u = [];
y = [];

while true
    line = strtrim(readline(s));

    % Si es la marca de fin, salimos
    if line == "END"
        disp('Recibido END desde Arduino.');
        break;
    end

    % Separar por comas
    parts = split(line, ",");
    if numel(parts) ~= 3
        % línea rara, la ignoramos
        continue;
    end

    % Convertir a números
    tv = str2double(parts(1));
    uv = str2double(parts(2));
    yv = str2double(parts(3));

    if any(isnan([tv, uv, yv]))
        continue;  % línea corrupta
    end

    t(end+1,1) = tv; %#ok<SAGROW>
    u(end+1,1) = uv; %#ok<SAGROW>
    y(end+1,1) = yv; %#ok<SAGROW>
end

% Cerrar puerto
clear s;

%% Ver datos crudos
figure;
subplot(2,1,1);
plot(t, u, '-'); grid on;
xlabel('t [s]'); ylabel('u [deg]');
title('Entrada (servo, offset desde el centro)');

subplot(2,1,2);
plot(t, y, '-'); grid on;
xlabel('t [s]'); ylabel('\theta_{pend} [deg]');
title('Salida (ángulo del péndulo)');

%% Construir objeto de identificación
% Estimamos el Ts a partir de los tiempos
dt = diff(t);
Ts_est = mean(dt);
fprintf('Ts estimado = %.5f s\n', Ts_est);

datos = iddata(y, u, Ts_est);

%% Identificar planta con 4 polos y 3 ceros
np = 4;  % polos
nz = 3;  % ceros

% tfest -> modelo discreto G(z) con Ts = Ts_est
sysd = tfest(datos, np, nz);

disp('---------------------------------------------');
disp('Modelo identificado G(z) (discreto):');
disp(sysd);
disp('---------------------------------------------');

% (Opcional) convertir a continuo si querés G(s)
% sysc = d2c(sysd, 'zoh');
% disp('Modelo aproximado G(s) (continuo):');
% disp(sysc);

%% Comparar modelo con datos medidos
figure;
compare(datos, sysd);
title('Comparación datos reales vs modelo identificado');

%% Respuesta al escalón de 20 grados
figure;
step(20 * sysd); % escalón de amplitud 20 (en deg de entrada)
grid on;
title('Respuesta de la planta al escalón de 20° en la entrada');

%% Respuesta al impulso de 30 grados
% En un modelo lineal, multiplicar por 30 escala la respuesta al impulso
figure;
impulse(30 * sysd);
grid on;
title('Respuesta de la planta al impulso de 30° en la entrada');