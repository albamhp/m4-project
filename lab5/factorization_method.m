function [Pproj,Xproj] = factorization_method(x1, x2)
%FACTORIZATION_METHOD computes a projective reconstruction with the 
% factorization method of Sturm and Triggs '1996
% This function returns an estimate of:
%       Pproj: 3*Ncam x 4 matrix containing the camera matrices
%       Xproj: 4 x Npoints matrix of homogeneous coordinates of 3D points
% 
    
    d_old = 100;
    maxIter = 1000;
    lambda = ones(2, length(x1));
    lambdas = kron(lambda, [1 1 1]');
    [x1, T1] = normalise2dpts(x1);
    [x2, T2] = normalise2dpts(x2);
    
    for iteration = 1:maxIter
        disp(lambda)

        % Create the design matrix M.
        M = lambdas.*[ x1; x2];
    
        % Rank 4
        [U,D,V] = svd(M,0);
        D = D(1:4,1:4);
        U = U(:,1:4);
        V = V(:,1:4)';
        
        Pproj = U*D;
        Xproj = V;
        
        
        % As a convergence criterion you may compute the Euclidean
        % distance (d) between data points and projected points in both images 
        % and stop when (abs(d - d_old)/d) < 0.1 where d_old is the distance
        % in the previous iteration.
        d = sqrt(sum(sum((M - (Pproj*Xproj)).^2)));
        if (abs(d - d_old)/d) < 0.1
            break;
        end
        d_old = d;
        
        
        for i= 1:2
            temp = Pproj((1:3)+(i-1)*3,:)*Xproj;
            lambda(i,:) = temp(3,:); 
        end
    end
end

