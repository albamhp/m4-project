function angle = get_angle(line1, line2)
    line1 = [line1(1)/line1(3), line1(2)/line1(3), 1];
    line2 = [line2(1)/line2(3), line2(2)/line2(3), 1];

    cos = dot(line1(1:2), line2(1:2)) / (norm(line1(1:2)) * norm(line2(1:2)));
    angle = rad2deg(acos(cos));
end