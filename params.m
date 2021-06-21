%
% Parameters for parking control
%
% developed by Soonkyu Jeong
% last modified Jun 21 2021
%

units;

% wheel base
Lb = 0.256*meter;

% local coordinates
O1 = struct('x',0*meter,'y',3*meter,'th', 0*deg);
O2 = struct('x',4*meter,'y',2*meter,'th',90*deg);

% road and parking space geometry
D2 = 0.7*meter;
W1 = 0.4*meter;
W2 = 1.0*meter;
S = 0.1*meter;

% goal point
Gp = struct('x', 4.0, 'y', 2.0);

% transformation matrix of O1 & O2
T01 = [1 0 O1.x; 0 1 O1.y; 0 0 1];
T02 = [0 -1 O2.x; 1 0 O2.y; 0 0 1];
T10 = inv(T01);
T20 = inv(T02);
T21 = T20 * T01;

% intersection points
Ax = 3.8;
Bx = 5.0;

% plotting range
x_max = 5*meter;
y_max = 4*meter;