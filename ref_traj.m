function [yr, dyr, ddyr] = ref_traj(x, is_backward)

l1y = 2.6; % Y
l2 = [1.929012346 -42.4382716 371.1419753 -1612.654321 3481.867284 -2986.569136]; % polynomial coefficient
l2p = [5 4 3 2 1] .* l2(1:end-1); % the first derivative coefficient
l2pp = [4 3 2 1] .* l2p(1:end-1); % the second derivative coefficient
l3y = 3.4; % Y
l4x = 4.0; % X
Ax = 3.8; % intersection points
Bx = 5.0;

n = length(x);
yr = zeros(size(x));
dyr = zeros(size(x));
ddyr = zeros(size(x));

if length(is_backward) == 1
    is_backward = logical(ones(size(x))*is_backward);
end

for i = 1:n
    if is_backward(i)
        yr(i) = l4x;
    else
        if x(i) < Ax
            yr(i) = l1y;
        elseif x(i) < Bx
            yr(i)   =   l2 * x(i).^[5 4 3 2 1 0]';
            dyr(i)  =  l2p * x(i).^[4 3 2 1 0]';
            ddyr(i) = l2pp * x(i).^[3 2 1 0]';
        else
            yr(i) = l3y;
        end
    end
end
