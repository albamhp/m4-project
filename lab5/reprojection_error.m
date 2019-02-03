function [error] = reprojection_error(P1, P2, X, x1, x2)
    x1p = P1*X;
    x2p = P2*X;
    
    error = computed2(x1, x1p) + computed2(x2, x2p);
end

function d2=computed2(a, b)
    d2 = ((a(1, :)./a(3, :))-(b(1, :)./b(3, :))).^2 ...
    + ((a(2, :)./a(3, :))-(b(2, :)./b(3, :))).^2;
end

