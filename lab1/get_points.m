function [p1, p2] = get_points(i, A)
    p1 = [A(i,1) A(i,2) 1]';
    p2 = [A(i,3) A(i,4) 1]';
end