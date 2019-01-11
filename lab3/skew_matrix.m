function m = skew_matrix(t)
    m = [0 -t(3) t(2) ; t(3) 0 -t(1) ; -t(2) t(1) 0];
end