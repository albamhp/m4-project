function error = geometric_error(F, x1, x2)
% GEOMETRIC_ERROR - calculate the geometric error
%
% Calculate the first order geometric error aproximation of an estimated
% fundamental matrix for a set of point correspondences. Also called
% Sampson error
%
% Usage: error = geometric_error(F, x1, x2)
%
% Arguments:
%   F  - 3x3 estimated fundamental matrix
%   x1 - 3xN array of 2D point correspondences of the first image in
%        homogeneous coordinates
%   x2 - 3xN array of 2D point correspondences of the second image in
%        homogeneous coordinates
%
% Returns:
%   error - 1xN array of the geometric error for each pair of
%           correspondences

    x2tFx1 = zeros(1,length(x1));
    for i = 1:length(x1)
	    x2tFx1(i) = x2(:,i)'*F*x1(:,i);
    end
    Fx1 = F*x1;
	Ftx2 = F'*x2; 
    error =  x2tFx1.^2 ./ ...
	     (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
end
