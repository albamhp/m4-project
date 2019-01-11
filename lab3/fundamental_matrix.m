function [F] = fundamental_matrix(x1, x2)
%FUNDAMENTAL_MATRIX Calculate the fundamental matrix using the normalised 8 point algorithm
%   x1 Set of points of the first image
%   x2 Set of points of the second image    
    [x1, T1] = normalise2dpts(x1);
    [x2, T2] = normalise2dpts(x2);
    
    
    W= [x2(1,:)'.*x1(1,:)'   x2(1,:)'.*x1(2,:)'  x2(1,:)' ...
         x2(2,:)'.*x1(1,:)'   x2(2,:)'.*x1(2,:)'  x2(2,:)' ...
         x1(1,:)'             x1(2,:)'            ones(size(x1,2),1) ]; 
     
    [U,D,V] = svd(W);   
    
    f = V(:,end);
    
    F_rank3 = reshape(f,3,3)';
    
    [Uf,Df,Vf] = svd(F_rank3);
    
    Df(end,end) = 0;
    
    F = Uf * Df * Vf';
   
end
