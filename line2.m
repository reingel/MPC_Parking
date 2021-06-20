% polynomial fitting

x1 = 3.8;
x2 = 5.0;

v1 = x1.^[5 4 3 2 1 0];
v2 = [5 4 3 2 1 0].*x1.^[4 3 2 1 0 0];
v3 = [5 4 3 2 1 0].*[4 3 2 1 0 0].*x1.^[3 2 1 0 0 0];
v4 = x2.^[5 4 3 2 1 0];
v5 = [5 4 3 2 1 0].*x2.^[4 3 2 1 0 0];
v6 = [5 4 3 2 1 0].*[4 3 2 1 0 0].*x2.^[3 2 1 0 0 0];


A = [v1; v2; v3; v4; v5; v6];
b = [2.6 0 0 3.4 0 0]';

c = A\b;
c = c';

disp(num2str(c,10))
disp(c*x1.^[5 4 3 2 1 0]')
disp(c*x2.^[5 4 3 2 1 0]')

% plotting

x = [0:0.1:6];
[yr, dyr, ddyr] = ref_traj(x, false);

figure(1)
plot(x,yr,'.-',x,dyr,'.-',x,ddyr,'.-');
grid on
