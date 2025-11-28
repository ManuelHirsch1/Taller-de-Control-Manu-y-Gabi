%% ============================================================
% 1. Configuración del puerto serie y warm-up
% =============================================================
close all; clear; clc;

% ⚙️ Ajustá el puerto según corresponda
arduinoPort = "COM9";          % Ejemplo: "COM3"
baudRate    = 115200;

% Crear puerto serie
s = serialport(arduinoPort, baudRate);

% Limpiar buffer inicial
flush(s);

% 🟡 Warm-up: el Arduino imprime mensajes de calibración y warm-up
disp("Esperando warm-up de la IMU...");
pause(3);   % Ajustar si hace falta

% Leer y descartar líneas de texto del setup/calibración
t_init = tic;
while toc(t_init) < 4   % 4 segundos descartando líneas
    try
        readline(s);
    catch
    end
end

disp("IMU calibrada. Iniciando adquisición...");

%% ============================================================
% 2. Parámetros de adquisición
% =============================================================
T_total = 10;     % Duración [s]
Ts      = 0.02;   % Periodo de muestreo esperado (20 ms)
N       = round(T_total / Ts);

% Prealocar vectores
t     = zeros(N,1);
theta = zeros(N,1);
yhat  = zeros(N,1);
u     = zeros(N,1);

%% ============================================================
% 3. Lectura del puerto serie
% =============================================================
disp("Grabando datos desde Arduino…");
for k = 1:N
    try
        line = readline(s);                       
        vals = str2double(split(line, ","));     

        % Formato Arduino: t_s,theta_deg,y_hat,u
        if numel(vals) >= 4
            t(k)     = vals(1);
            theta(k) = vals(2);
            yhat(k)  = vals(3);
            u(k)     = vals(4);
        elseif k > 1
            % Si llega línea incompleta se repite muestra anterior
            t(k)     = t(k-1) + Ts;
            theta(k) = theta(k-1);
            yhat(k)  = yhat(k-1);
            u(k)     = u(k-1);
        end

    catch
        % Error en lectura → repetir último valor
        if k > 1
            t(k)     = t(k-1) + Ts;
            theta(k) = theta(k-1);
            yhat(k)  = yhat(k-1);
            u(k)     = u(k-1);
        end
    end
end

disp("Grabación finalizada.");
clear s;   % cerrar puerto

%% ============================================================
% 4. Limpieza de datos
% =============================================================
valid = t > 0;
t     = t(valid);
theta = theta(valid);
yhat  = yhat(valid);
u     = u(valid);

%% ============================================================
% 5. Gráficos
% =============================================================

figure('Name','Ángulo del péndulo','NumberTitle','off');
plot(t, theta, 'LineWidth',1.5);
hold on;
plot(t, yhat, '--','LineWidth',1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('\theta [°]');
title('Respuesta real y estimada del péndulo');
legend('Medición IMU','Estado estimado (observador)');
set(gca,'FontSize',12);
set(gcf,'Color','w');

figure('Name','Acción de control','NumberTitle','off');
plot(t, u,'LineWidth',1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('u [°]');
title('Acción de control aplicada al servomotor');
set(gca,'FontSize',12);
set(gcf,'Color','w');
