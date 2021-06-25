%
% Reference trajectory for parking control
%
% developed by Soonkyu Jeong
% last modified Jun 26 2021
%

function [yr, dyr, ddyr] = ref_traj(k, xs)

params;

l1y = 2.7; % Y
l2 = [1.687885802 -37.13348765 324.7492284 -1411.072531 3046.633873 -2612.822994]; % polynomial coefficient
l2p = [5 4 3 2 1] .* l2(1:end-1); % the first derivative coefficient
l2pp = [4 3 2 1] .* l2p(1:end-1); % the second derivative coefficient
l3y = 3.4; % Y
l4x = 4.0; % X

n = length(xs);
yr = zeros(size(xs));
dyr = zeros(size(xs));
ddyr = zeros(size(xs));

if k == 1 % forward
    for i = 1:n
        if xs(i) < Ax
            yr(i) = l1y - O1.y;
        elseif xs(i) < Bx
            yr(i)   =   l2 * xs(i).^[5 4 3 2 1 0]' - O1.y;
            dyr(i)  =  l2p * xs(i).^[4 3 2 1 0]';
            ddyr(i) = l2pp * xs(i).^[3 2 1 0]';
        else
            yr(i) = l3y - O1.y;
        end
    end
else % backward
    for i = 1:n
        yr(i) = O2.x - l4x;
    end
end
