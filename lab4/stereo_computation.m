function dist = stereo_computation(leftImage, rightImage, minDisp, maxDisp, winSize, cost_function, weights)

    leftImage = rgb2gray(leftImage);
    rightImage = rgb2gray(rightImage);
    [m,n] = size(leftImage);
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
                
                if (weights==1)
                    for indexQ=1:winSize^2
                        [ip, jp] = ind2sub(winSize^2, indexQ);
                        p = winLeft(winHalf+1,winHalf+1);
                        q = winRight(indexQ);
                        w = -(abs(p-q)/12)-(abs(sqrt((i-ip)^2+(j-jp)^2))/17.5);
                    end
                end
                
                if strcmp('SSD', cost_function)
                    cost = sum((abs(winLeft(:) - winRight(:))).^2 );
                elseif strcmp('SAD', cost_function)
                    cost = sum(abs(winLeft(:) - winRight(:)));
                elseif strcmp('NCC', cost_function)
                    i1 = winLeft-mean2(winLeft);
                    i2 = winRight-mean2(winRight);
                    cost = -sum(i1.*i2)/(sqrt(sum(i1.^2)).*sqrt(sum(i2.^2)));
                        
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




