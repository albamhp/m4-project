function error = algebraic_error(F_est, p1, p2)
    error = zeros(length(p1), 1);
    for i = 1:length(p1)
       error(i) = abs(p2(:, i)' * F_est * p1(:, i));
    end
end

