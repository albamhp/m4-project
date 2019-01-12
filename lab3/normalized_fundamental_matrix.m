function [F] = normalized_fundamental_matrix(x1, x2)
% NORMILIZED_FUNDAMENTAL_MATRIX - calculate fundamental matrix
%
% Calculate the fundamental matrix using the normalized 8 point algorithm
%   
% Usage: F = normalized_fundamental_matrix(x1, x2)
%
% Arguments:
%   x1 - 3xN array of 2D point correspondences of the first image in 
%        homogeneous coordinates 
%   x2 - 3xN array of 2D point correspondences of the second image in 
%        homogeneous coordinates
%
% Returns:
%   F - Fundamental matrix

    [nx1, T1] = normalise2dpts(x1);
    [nx2, T2] = normalise2dpts(x2);
    
    F = fundamental_matrix(nx1, nx2);
    
    F = T2' * F * T1;
end