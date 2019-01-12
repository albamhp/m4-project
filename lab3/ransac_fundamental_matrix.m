function [F, inliers] = ransac_fundamental_matrix(x1, x2, error_function, th, ite)
% RANSAC_FUNDAMENTAL_MATRIX - calculate fundamental matrix
%
% Calculate the fundamental matrix using RANSAC and normalized 8 point 
% algorithm
%   
% Usage: [F, inliers] = ransac_fundamental_matrix(x1, x2, error_function, th, ite)
%
% Arguments:
%   x1             - 3xN array of 2D point correspondences of the first
%                    image in homogeneous coordinates 
%   x2             - 3xN array of 2D point correspondences of the second
%                    image in homogeneous coordinates
%   error_function - Error function used to calculate the inliers
%   th             - Threshold to decide whether a pair is an inlier or
%                    outlier
%   ite            - Number of iterations
%
% Returns:
%   F       - Fundamental matrix
%   inliers - Indices of the point correspondences used as inliers

    bestInliers = [];

    for i = 1:ite
        points = randomsample(length(x1), 8);
        F_est = normalized_fundamental_matrix(x1(:,points), x2(:,points));
        
        err = error_function(F_est, x1, x2);
        currentInliers = find(err <= th);
        if (length(currentInliers) > length(bestInliers))
           bestInliers = currentInliers;
        end
    end

    F = normalized_fundamental_matrix(x1(:, bestInliers), x2(:, bestInliers));
    inliers = bestInliers;

end

function item = randomsample(num_points, num_samples)
    a = 1:num_points; 
    item = zeros(1,num_samples);    
    for i = 1:num_samples
        % Generate random value in the appropriate range 
        r = ceil((num_points-i+1).*rand);
        item(i) = a(r);       % Select the rth element from the list
        a(r)    = a(end-i+1); % Overwrite selected element
    end
end 
