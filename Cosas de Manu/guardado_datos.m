%% === 1. Configuración del puerto serie ===
close all; clear; clc;

% ⚙️ Ajustá el puerto COM según tu PC
arduinoPort = "COM3";     % Ejemplo: "COM3" o "/dev/ttyUSB0"
baudRate    = 115200;

% Crear puerto serie y limpiar buffer
s = serialport(arduinoPort, baudRate);
flush(s);
pause(2);  % Esperar a que Arduino arranque

% === Parámetros de adquisición ===
T_total = 10;        % Duración total del registro [s]
Ts      = 0.02;      % Periodo de muestreo estimado [s] (ej. 50 Hz)
N       = round(T_total / Ts);

% Prealocar vectores
t = zeros(N,1);
y = zeros(N,1);

disp("Grabando datos desde Arduino...");
tic;

%% === 2. Lectura del puerto serie ===
for k = 1:N
    try
        line = readline(s);                     % leer una línea
        vals = str2double(split(line, ","));    % separar por coma
        if numel(vals) >= 2
            t(k) = vals(1);
            y(k) = vals(2);
        elseif k > 1
            % si no se leyó bien la línea, repetir último valor
            t(k) = t(k-1) + Ts;
            y(k) = y(k-1);
        end
    catch
        % ante error de lectura, mantener último valor
        if k > 1
            t(k) = t(k-1) + Ts;
            y(k) = y(k-1);
        end
    end
end

disp("Grabación finalizada.");
clear s;  % cerrar puerto serie
toc;

%% === 3. Limpieza de datos ===
valid = t > 0;
t = t(valid);
y = y(valid);

%% === 4. Graficar ===
figure('Name','Datos Arduino','NumberTitle','off');
plot(t, y, 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Salida y (°)');
title('Ángulo medido desde Arduino');
set(gca, 'FontSize', 12);
set(gcf, 'Color', 'w');

%% === 5. Guardar resultados ===
outName = "registro_" + datestr(now, "HHMMSS");
T = table(t, y);
writetable(T, outName + ".csv");
save(outName + ".mat", "t", "y", "Ts");
disp("Datos guardados en: " + outName + ".csv");
