clc; clear ; close all
format long g

%% Planta continua
z = [0, -8];
p = [ -114, -1.44, -0.5 + 7.5i, -0.5 - 7.5i];
k = -90;
Ts = 20e-3; % Tiempo de muestreo

P = zpk(z,p,k);

%% Conviento a ss
sysc = ss(P);
A = sysc .A;
B = sysc .B;
C = sysc .C;
D = sysc .D;


% Discretizamos del sistema
sysd = c2d(sysc , Ts , 'zoh');
Ad = sysd .A;
Bd = sysd .B;
Cd = sysd .C;
Dd = sysd .D;


%% Verificamos la controlabilidad y observabilidad
assert ( rank ( ctrb (A,B)) == size (A ,1) , 'Sistema no controlable en continuo ')
assert ( rank ( obsv (A,C)) == size (A ,1) , 'Sistema no observable en continuo ')
assert ( rank ( ctrb (Ad ,Bd)) == size (Ad ,1) , 'Sistema no controlable en discreto')
assert ( rank ( obsv (Ad ,Cd)) == size (Ad ,1) ,  'Sistema no observable en discreto ')

%% Definición de polos deseados para controladores
% Polos deseados en continuo ( controlador )
p1c = -66;
p2c = -88;
p3c = -3 + 6i;
p4c = -3 - 6i;
polyc = [p1c; p2c; p3c; p4c ]; % polos deseados en s

% Conversión a z ( discreto ) usando z = exp(s*Ts)
polyd = exp( polyc * Ts); % polos deseados en z

% Cálculo de ganancias del controlador
Kc = acker (A, B, polyc ); % controlador en s
Kd = acker (Ad , Bd , polyd ); % controlador en z

%% Polos deseados para observador
% Para el observador propongo ( todos los polos en s = pec )
pec = -100;
polyec = [pec pec pec pec ];
Lc = acker (A.', C.', polyec ).'; % observador en continuo

% Versión discreta
polyed = exp( polyec * Ts);
Ld = acker (Ad.', Cd.', polyed ).'; % observador en discreto

%% Armado del observador
% Observador continuo
Aoc = A - Lc*C;
Boc = [B Lc ];
Chat = eye( size (A));
Doc = zeros ( size (A ,1) , 2);
observadorc = ss(Aoc , Boc , Chat , Doc);

% Observador discreto
Ao = Ad - Ld*Cd;
Bo = [Bd Ld ];
observadord = ss(Ao , Bo , Chat , Doc);

% Observador para Arduino
observadordd = c2d( observadorc , Ts , 'zoh ');
[Aodd ,Bodd ,Codd , Dodd ] = ssdata ( observadordd );

%% Mostrar resultados
disp ('Ganancia del controlador discreto (Kd):');
disp (Kd);
disp ('Matriz A del sistema discretizado ( Aodd ):');
disp ( Aodd );
disp ('Matriz B del sistema discretizado ( Bodd ):');
disp ( Bodd );

