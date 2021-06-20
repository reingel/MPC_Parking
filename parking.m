%
% Parking control with MPC
%
% developed by Soonkyu Jeong
% last modified Jun 19, 2021
%

clear

params;


% simulation parameters
V = 200*mm/sec;
Dx = 0.1*meter;
N = 6;

Dt = 1*sec;
t = [0:Dt:100];
n = length(t);

z1 = zeros(1,n+1); z1(1) = O1.x;
z23 = zeros(2,n+1); z23(:,1) = [O1.y, 0*deg]';
u = zeros(1,n);

% video object definition
MAP1 = figure(111);
set(MAP1,'Color','White');
clf
hold on

% plot reference trajectory
x = [0:0.1:6];
yr = ref_traj(x, false);
plot(x,yr,'m--');
plot([O2.x O2.x],[O2.y O2.y+2*D2],'m--')
plot([O1.x x_max x_max O2.x+W1/2 O2.x+W1/2 O2.x-W1/2 O2.x-W1/2 O1.x], ...
    [O1.y+W2/2 O1.y+W2/2 O1.y-W2/2 O1.y-W2/2 O1.y-W2/2-D2 O1.y-W2/2-D2 ...
    O1.y-W2/2 O1.y-W2/2],'k');
plot(O1.x,O1.y,'>b')
plot(O2.x,O2.y,'vb')
grid on
axis equal
axis([-0.2 x_max 1.5 y_max])

nb = 0; % steps for backward driving
i_sp = 1; % starting index for backward driving

% simulation
for i = 1:n
    % initialization
    z0 = z23(:,i);
    u0 = u(max(1,i-1));
    
    if z1(i) > Ax % consider backward driving
        nb = min(N,nb + 1);
        i_sp = min(i + nb,n+1);
        x = [z1(i) + Dx*[1:N-nb]'; z23(1,i_sp) - O2.y - Dx*[1:nb]'];
        is_backward = [logical(zeros(1,N-nb)) ones(1,nb)];
        Yd = ref_traj(x, is_backward);
    else
        % desired Y
        x = z1(i) + Dx*[1:N]';
        is_backward = logical(zeros(1,N));
        Yd = ref_traj(x, is_backward);
    end
    
    U = qp(N, Yd, z0, u0, is_backward);
    
    % selection of first control input
    u(i) = U(1);
    
    % dynamics
    if is_backward(1)
        z1(i+1) =z1(i) - Dx;
        A = [1 -Dx; 0 1];
        B = [Dx^2/2; -Dx];
        z23(:,i+1) = A*z23(:,i) + B*u(i);
    else
        z1(i+1) =z1(i) + Dx;
        A = [1 Dx; 0 1];
        B = [Dx^2/2; Dx];
        z23(:,i+1) = A*z23(:,i) + B*u(i);
    end
    
    figure(111)
    if is_backward(1)
        plot(z1(1:i-nb),z23(1,1:i-nb),'b')
        plot(z23(i-nb:i) - O2.y, O2.x - z1(1,i-nb:i),'r')
        d2goal = sqrt((z23(i-nb:i) - O2.y)^2 + (O2.x - z1(1,i-nb:i))^2);
    else
        plot(z1(1:i),z23(1,1:i),'b')
        d2goal = sqrt(z1(i)^2 + z23(1,i)^2);
    end
    
    if d2goal < S
        break
    end
end


