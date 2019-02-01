function [V] = vanishing_point(xo1, xf1, xo2, xf2)

    xa = cross(xo1, xf1);
    xb = cross(xo2, xf2);
    
    xa = xa ./ xa(3);
    xb = xb ./ xb(3);

    V = cross(xa, xb);
    
end
