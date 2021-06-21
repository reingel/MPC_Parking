%
% Parking control with MPC
%
% developed by Soonkyu Jeong
% last modified Jun 21, 2021
%

clear
params;

% simulation parameters
N = 6; % no. of prediction steps
Dx = 0.1*meter;
Dt = 1*sec;
t = [0:Dt:100];
n = length(t);

% reference trajectory
xr = [0:0.1:6];
yr = ref_traj(1, xr) + O1.y;

% state equations for forward and backward
A1 = [1 Dx; 0 1];
B1 = [Dx^2/2; Dx];
A2 = [1 -Dx; 0 1];
B2 = [Dx^2/2; -Dx];
C = [1 0];

% initialization for state & input variables
z1 = zeros(1,n+1); z1(1) = O1.x;
z23 = zeros(2,n+1); z23(:,1) = [O1.y, 0*deg]';
u = zeros(1,n);

nb = 0; % steps for backward driving

% video object definition
MAP1 = figure(1);
set(MAP1,'Color','White');
vidObj = VideoWriter('mpc_parking');
vidObj.FrameRate = 20 ;
open(vidObj);

% simulation
for i = 1:n
    Xi = z1(i);
    Yi = z23(1,i);
    THi = z23(2,i);
    
    u0 = u(max(i-1,1)); % minimum index is 1
    
    if Xi < Ax % car is far from the parking space
        z0 = [z23(1,i) - O1.y; z23(2,i)];
        xs = Xi + Dx*[1:N]';
        Yd = ref_traj(1,xs);
        U = qp(N, A1, B1, C, Yd, z0, u0); % quadprog for forward
    else % car is near the parking space
        nb = min(N, nb+1);
        U = [];
        if nb < N % forward
            z0 = [z23(1,i) - O1.y; z23(2,i)];
            xs = Xi + Dx*[1:N-nb]';
            Yd = ref_traj(1,xs);
            U = [U; qp(N-nb, A1, B1, C, Yd, z0, u0)]; % quadprog for forward
        end
        if nb > 0 % backward
            z0 = [O2.x - (Xi + Dx*(N-nb)); z23(2,i) - O2.th];
            if ~isempty(U)
                u0 = U(end);
            end
            xs = Yi - O2.y - Dx*[1:nb]';
            Yd = ref_traj(2, xs);
            U = [U; qp(nb, A2, B2, C, Yd, z0, u0)]; % quadprog for backward
        end
    end
    
    u(i) = U(1);
    
    if nb < N % u(i) is for forward
        % world coord. to O1 coord.
        XY = [Xi; Yi; 1];
        XY1 = T10 * XY;
        X1 = XY1(1);
        Y1 = XY1(2);
        TH = z23(2,i);
        Y1TH = [Y1; TH];
        
        % forward dynamics
        X1p = X1 + Dx;
        Y1THp = A1*Y1TH + B1*u(i);
        Y1p = Y1THp(1);
        THp = Y1THp(2);
        
        % O1 coord. to world coord.
        XY1p = [X1p; Y1p; 1];
        XYp = T01 * XY1p;
        Xp = XYp(1);
        Yp = XYp(2);
        YTHp = [Yp; THp];
        
        % save state variables
        z1(i+1) = Xp;
        z23(:,i+1) = YTHp;
    else % nb == N, u(i) is for backward
        % world coord. to O2 coord.
        XY = [Xi; Yi; 1];
        XY2 = T20 * XY;
        X2 = XY2(1);
        Y2 = XY2(2);
        TH = z23(2,i) - O2.th;
        Y2TH = [Y2; TH];
        
        % backward dynamics
        X2p = X2 - Dx;
        Y2THp = A2*Y2TH + B2*u(i);
        Y2p = Y2THp(1);
        THp = Y2THp(2) + O2.th;
        
        % O2 coord. to world coord.
        XY2p = [X2p; Y2p; 1];
        XYp = T02 * XY2p;
        Xp = XYp(1);
        Yp = XYp(2);
        YTHp = [Yp; THp];
        
        % save state variables
        z1(i+1) = Xp;
        z23(:,i+1) = YTHp;
    end
    
    % animation
    figure(1)
    clf
    hold on
    plot(xr,yr,'m--');
    plot([O2.x O2.x],[O2.y O2.y+2*D2],'m--')
    plot([O1.x x_max x_max O2.x+W1/2 O2.x+W1/2 O2.x-W1/2 O2.x-W1/2 O1.x], ...
        [O1.y+W2/2 O1.y+W2/2 O1.y-W2/2 O1.y-W2/2 O1.y-W2/2-D2 O1.y-W2/2-D2 ...
        O1.y-W2/2 O1.y-W2/2],'k');
    plot(O1.x,O1.y,'>b')
    plot(O2.x,O2.y,'vb')
    grid on
    axis equal
    axis([-0.2 x_max 1.5 y_max])

    plot(z1(1:i),z23(1,1:i),'b')
    Rcar = [cos(THi) -sin(THi); sin(THi) cos(THi)];
    Pcar = [Xi; Yi];
    box_car = Rcar*[-0.1 Lb+0.1 Lb+0.1 -0.1 -0.1; 0.1 0.1 -0.1 -0.1 0.1] + Pcar;
    plot(box_car(1,:),box_car(2,:));
    currFrame = getframe(MAP1);
    writeVideo(vidObj,currFrame);
    
    % termination condition
    if Yi < Gp.y
        break
    end
end

close(vidObj);
