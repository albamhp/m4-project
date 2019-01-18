function dist = stereo_computation(leftImage, rightImage, minDisp, maxDisp, winSize, cost_function)

    leftImage = rgb2gray(leftImage);
    rightImage=rgb2gray(rightImage);
    [m,n]=size(leftImage);
    dist = zeros(m,n);
    winHalf = floor(winSize/2);

    for i=1+winHalf:m-winHalf
        for j=1+winHalf:n-winHalf
            bestCost = inf;
            best_win = 0;
            
            winLeft = leftImage(i-winHalf:i+winHalf,j-winHalf:j+winHalf);
            for win=-maxDisp:maxDisp
                if j + win - winHalf <= 0 || j + win + winHalf > n || abs(win) < minDisp
                   continue; 
                end
                winRight = rightImage(i-winHalf:i+winHalf,j+win-winHalf:j+win+winHalf);
                if strcmp('SSD', cost_function)
                    cost = sum( (winLeft(:) - winRight(:)).^2 );
                elseif strcmp('NCC', cost_function)
                    cost = sum(abs(winLeft(:) - winRight(:)));
                else
                    error('Invalid cost function')
                end

                if cost < bestCost
                    best_win = win;
                    bestCost = cost;
                end
            end
            dist(i,j) = abs(best_win);
        end    
    end
end




