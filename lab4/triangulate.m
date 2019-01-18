function [pts3D] = triangulate(x1, x2, P1, P2, imsize)

%   Function triangulate.m that performs a triangulation
%   with the homogeneous algebraic method (DLT)
%
%       The entries are (x1, x2, P1, P2, imsize), where:
%           - x1, and x2 are the Euclidean coordinates of two matching 
%             points in two different images.
%           - P1 and P2 are the two camera matrices
%           - imsize is a two-dimensional vector with the image size

    npts=size(x1, 2);
    pts3D=zeros(4, npts);

    H1=[2/imsize(1) 0 -1
        0 2/imsize(2) -1
        0 0 1];
    H2=[2/imsize(1) 0 -1
        0 2/imsize(2) -1
        0 0 1];
     
    % left multiply Ps and points
    nP1=H1*P1;
    nP2=H2*P2;
    nx1= H1 * [x1; ones(1, length(x1))];
    nx2= H2 * [x2; ones(1, length(x2))];
    
    % at this point, normalization has been performed with the H's above
    for i=1:npts
        A=[nx1(1:2,i)*nP1(3,:)-nP1(1:2,:);
           nx2(1:2,i)*nP2(3,:)-nP2(1:2,:)];
        % Linear-Eigen
        [~, ~, V]=svd(A);
        pts3D(:,i)=V(1:4, 4) ./ V(4, 4);
    end

end