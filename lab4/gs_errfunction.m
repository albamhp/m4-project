function [Y] = gs_errfunction(P0, x, xp)
% gs_errfunction(P0, x, xp) returns the reprojection error for the image.
% P0 is a vector containing the homography and the projections points, 
% x are the point in the first image and xp are the points in the
% projection image. x and xp are of size 2xN and are assumed to be already
% normalized.

    H = reshape(P0(1:9), 3, 3);
    
    xh = P0(10:end);
    xh = reshape(xh, 2, length(P0(10:end))/2);
    xh = [xh; ones(1, length(P0(10:end))/2)];
    xhp = H * xh;
    xhp = xhp./(repmat(xhp(3,:),3,1));
    
    xh = xh(1:2, :);
    xhp = xhp(1:2, :);
    
    i = 1:length(x)/2;
    e1 = (x(1,i) - xh(1,i)).^2 + (x(2,i) - xh(2,i)).^2;
    e2 = (xp(1,i) - xhp(1,i)).^2 + (xp(2,i) - xhp(2,i)).^2;
    
    Y = e1 + e2; % error in both sides
    Y = double(Y);
end

