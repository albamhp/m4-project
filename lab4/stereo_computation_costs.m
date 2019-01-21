function dist = stereo_computation_costs(leftImage, rightImage, minDisp, maxDisp, winSize, cost_function)

    leftImage = int16(rgb2gray(leftImage));
    rightImage = int16(rgb2gray(rightImage));
    [m,n] = size(leftImage);
    dist = ones(m,n, maxDisp - minDisp + 1) .* inf;
    winHalf = floor(winSize/2);

    for i=1+winHalf:m-winHalf
        for j=1+winHalf:n-winHalf
           
            winLeft = leftImage(i-winHalf:i+winHalf,j-winHalf:j+winHalf);
            cost_i = 1;
            for win=minDisp:maxDisp
                if j + win - winHalf <= 0 || j + win + winHalf > n
                   continue; 
                end
                winRight = rightImage(i-winHalf:i+winHalf,j+win-winHalf:j+win+winHalf);
                if strcmp('SSD', cost_function)
                    cost = sum((abs(winLeft(:) - winRight(:))).^2 );
                elseif strcmp('SAD', cost_function)
                    cost = sum(abs(winLeft(:) - winRight(:)));
                elseif strcmp('NCC', cost_function)
                    i1 = winLeft-mean2(winLeft);
                    i2 = winRight-mean2(winRight);
                    cost = 1-sum(sum(i1.*i2))/(sqrt(sum(sum(i1.^2))).*sqrt(sum(sum(i2.^2))));
                else
                    error('Invalid cost function')
                end
                dist(i,j, cost_i) = cost;
                cost_i = cost_i + 1;
            end
        end    
    end
end




