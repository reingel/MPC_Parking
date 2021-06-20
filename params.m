%
% Parameters for parking control
%
% developed by Soonkyu Jeong
% last modified Jun 19 2021
%
% reference: 2013 Oyama
%


units;

x0 = 1.0*meter;
y0 = 3.0*meter;
th0 = 0.0*deg;

V = 0.2*meter/sec;
Lb = 0.256*meter;

Q = diag([5.5, 1.0]);
R = 1.0e-3;
Qfin = diag([5.5, 1.0]);
Qp = 5.0;
k1 = struct('fwd',4.0,'bwd', 4.0); % [forward, backward]
k2 = struct('fwd',4.0,'bwd',-4.0);

O1 = struct('x',0*meter,'y',3*meter,'th', 0*deg);
O2 = struct('x',4*meter,'y',2*meter,'th',90*deg);
x_max = 5*meter;
y_max = 4*meter;

D2 = 0.7*meter;
W1 = 0.4*meter;
W2 = 1.0*meter;
S = 0.1*meter;
Gp = [4.0, 2.0];


Ax = 3.8; % intersection points
Bx = 5.0;