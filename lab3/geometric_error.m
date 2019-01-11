function error = geometric_error(F, p1, p2)
    p2tFp1 = zeros(1,length(p1));
    for i = 1:length(p1)
	    p2tFp1(i) = p2(:,i)'*F*p1(:,i);
    end
    Fp1 = F*p1;
	Ft2 = F'*p2; 
    error =  p2tFp1.^2 ./ ...
	     (Fp1(1,:).^2 + Fp1(2,:).^2 + Ft2(1,:).^2 + Ft2(2,:).^2);
end
