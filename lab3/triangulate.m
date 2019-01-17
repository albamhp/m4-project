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
    pts3D=zeros(3, npts);

    H1=[2/imsize(1, 1) 0 -1
        0 2/imsize(2, 1) -1
        0 0 1];
    H2=[2/imsize(1, 2) 0 -1
        0 2/imsize(2, 2) -1
        0 0 1];
     
    % left multiply Ps and points
    nP1=H1*P1;
    nP2=H2*P2;
    nx1=H1(1:2, 1:2)*x1 + repmat(H1(1:2, 3), 1, npts);
    nx2=H2(1:2, 1:2)*x2 + repmat(H2(1:2, 3), 1, npts);
    
    % at this point, normalization has been performed with the H's above
    for i=1:npts
        A=[nx1(:,i)*nP1(3,:)-nP1(1:2,:); nx2(:,i)*nP2(3,:)-nP2(1:2,:)];
        % Linear-Eigen
        [~, ~, V]=svd(A, 0);
        pts3D(:,i)=V(1:3, 4)./V(4, 4);
    end

end