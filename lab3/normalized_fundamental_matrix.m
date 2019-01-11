function [F] = normalized_fundamental_matrix(x1, x2)
    [nx1, T1] = normalise2dpts(x1);
    [nx2, T2] = normalise2dpts(x2);
    
    F = fundamental_matrix(nx1, nx2);
    
    F = T2' * F * T1;
end