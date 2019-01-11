function [F, inliers] = ransac_fundamental_matrix(p1, p2, error_function, th, ite)
%FUNDAMENTAL_MATRIX_RANSAC Calculate the fundamental matrix using RANSAC
%   x1 Set of points of the first image
%   x2 Set of points of the second image  
%   th thershold 

    bestInliers = [];

    for i = 1:ite
        points = randomsample(length(p1), 8);
        F_est = normalized_fundamental_matrix(p1(:,points), p2(:,points));
        
        err = error_function(F_est, p1, p2);
        currentInliers = find(err <= th);
        if (length(currentInliers) > length(bestInliers))
           bestInliers = currentInliers;
        end
    end

    F = normalized_fundamental_matrix(p1(:, bestInliers), p2(:, bestInliers));
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
