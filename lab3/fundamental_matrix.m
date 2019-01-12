function [F] = fundamental_matrix(x1, x2)
% FUNDAMENTAL_MATRIX - calculate fundamental matrix
%
% Calculate the fundamental matrix using the 8 point algorithm
%
% Usage: F = fundamental_matrix(x1, x2)
%
% Arguments:
%   x1 - 3xN array of 2D point correspondences of the first image in 
%        homogeneous coordinates 
%   x2 - 3xN array of 2D point correspondences of the second image in 
%        homogeneous coordinates
%
% Returns:
%   F - Fundamental matrix
     
    W = kron(x2', [1 1 1]) .* [x1; x1; x1]';
     
    [~, ~, V] = svd(W);   
    
    f = V(:,end);
    
    F_rank3 = reshape(f,3,3)';
    
    % Make the matrix rank 2
    [Uf,Df,Vf] = svd(F_rank3);
    Df(end,end) = 0;
    F = Uf * Df * Vf';
end
