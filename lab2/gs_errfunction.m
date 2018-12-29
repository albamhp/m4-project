function [Y] = gs_errfunction(P0, x, xp)
%GS_ERRFUNCTION Summary of this function goes here
%   Detailed explanation goes here
% TODO

    H = reshape(P0(1:9),3,3);
    
    xh = P0(10:end);
    xh = reshape(xh, 2, length(P0(10:end))/2);
    xh = [xh; ones(1, length(P0(10:end))/2)];
    xhp = H * xh;
    xhp = xhp./(repmat(xhp(3,:),3,1));
    
    xh = xh(1:2, :);
    xhp = xhp(1:2, :);
    
    i = 1:length(x)/2;
    dworld = (x(1,i) - xh(1,i)).^2 + (x(2,i) - xh(2,i)).^2;
    dimage = (xp(1,i) - xhp(1,i)).^2 + (xp(2,i) - xhp(2,i)).^2;
    
    Y = dworld + dimage; % error in both images
    Y = double(Y);
end

