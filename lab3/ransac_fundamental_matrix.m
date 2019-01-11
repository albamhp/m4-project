function [F, inliers] = ransac_fundamental_matrix(p1, p2, th)
%FUNDAMENTAL_MATRIX_RANSAC Calculate the fundamental matrix using RANSAC
%   x1 Set of points of the first image
%   x2 Set of points of the second image  
%   th thershold 

maxInliers = 0;
F = zeros(3,3);

for i = 1:1000
    F_est = fundamental_matrix(p1,p2);
    err = sum((p2 .* (F_est * p1)),2);
    currentInliers = size( find(abs(err) <= th) , 1);
    if (currentInliers > maxInliers)
       F = F_est; 
       maxInliers = currentInliers;
    end    
end

err = sum((p2 .* (F * p1)),2);
[Y,I]  = sort(abs(err),'ascend');
inliers = I;

end
