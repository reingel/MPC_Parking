%
% Quadratic programming for parking control
%
% developed by Soonkyu Jeong
% last modified Jun 26 2021
%

function U = qp(N, A, B, C, Yd, z0, u0)

params;

% matrices to compute
G = zeros(N,2);
F = zeros(N,1);
H = zeros(N);
D = diag([ones(1,N-1) 0]) + diag(-1*ones(1,N-1),1);

% 1) input constraint
Au1 = [eye(N); -eye(N)];
bu1 = tan(30*deg)/(Lb*cos(0*deg)^3) * ones(2*N,1);

% 2) output constraint
Ay = [eye(N); -eye(N)];
by = (W2-S)*ones(2*N,1);

for i = 1:N
    G(i,:) = C*A^i;
    F(i) = C*A^(i-1)*B;
    if i > 1
        row = zeros(1,N);
        for j = i:-1:2
            row(j-1) = C*A^(j-2)*B;
        end
        H(i,:) = row;
    end
end

% matrices for a matrix-form of cost
r = 1; % gamma for control energy weight
Q = r*D'*D + H'*H;

Au = Ay * H;
At = [Au1; Au];

% computation of f
f = H' * (G*z0 + F*u0 - Yd);

% total constraint
bu = by - Ay*(G*z0 + F*u0);
bt = [bu1; bu];

% optimization with QP (Quadratic Programming)
U = quadprog(Q,f,At,bt);

