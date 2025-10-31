% === Parámetros ===
port = "COM3";    % <--- CAMBIAR por tu puerto (en macOS: "/dev/tty.usbmodemXXXX")
baud = 115200;
t_log_s = 12;     % tiempo máximo de log (s) - ajustá según necesites

% === Abrir puerto ===
if exist('s','var') && isa(s,'serialport'); try, clear s; end; end
s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

% === Leer encabezado ===
hdr = readline(s);   % "t_s,theta_deg,phi_ref_deg"
fprintf("Header: %s\n", strtrim(hdr));

% === Búfer de datos ===
t = []; theta = []; phi_ref = [];

t_start = tic;
while toc(t_start) < t_log_s
    if s.NumBytesAvailable > 0
        ln = readline(s);
        parts = split(strtrim(ln), ",");
        if numel(parts) == 3
            ti  = str2double(parts{1});
            th  = str2double(parts{2});
            phi = str2double(parts{3});
            if isfinite(ti) && isfinite(th) && isfinite(phi)
                t(end+1,1) = ti; %#ok<*AGROW>
                theta(end+1,1) = th;
                phi_ref(end+1,1) = phi;
            end
        end
    end
end

% === Guardar CSV ===
T = table(t, theta, phi_ref, 'VariableNames', {'t_s','theta_deg','phi_ref_deg'});
outcsv = "log_escalon.csv";
writetable(T, outcsv);
fprintf("Guardado: %s (%d muestras)\n", outcsv, height(T));

% === Graficar ===
figure; 
plot(T.t_s, T.theta_deg, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\theta [deg]'); title('Respuesta \theta(t)');
figure;
plot(T.t_s, T.phi_ref_deg, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\phi_{ref} [deg]'); title('Entrada escalón \phi_{ref}(t)');
