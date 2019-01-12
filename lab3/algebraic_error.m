function error = algebraic_error(F, x1, x2)
% ALGEBRAIC_ERROR - calculate the algebraic error
%
% Calculate the algebraic error of an estimated fundamental matrix. The
% result of p2'*F*p1 should be 0 if F was perfect, so any deviation from
% it will be the error.
%
% Usage: error = algebraic_error(F, x1, x2)
%
% Arguments:
%   F  - 3x3 estimated fundamental matrix
%   x1 - 3xN array of 2D point correspondences of the first image in
%        homogeneous coordinates
%   x2 - 3xN array of 2D point correspondences of the second image in
%        homogeneous coordinates
%
% Returns:
%   error - 1xN array of the algebraic error for each pair of
%           correspondences

    error = zeros(1, length(x1));
    for i = 1:length(x1)
       error(i) = abs(x2(:, i)' * F * x1(:, i));
    end
end

