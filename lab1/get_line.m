function l1 = get_line(p1, p2)
    coefficients = polyfit([p1(1), p2(1)], [p1(2), p2(2)], 1);
    l1 = [coefficients(1) -1 coefficients(2)];
end
