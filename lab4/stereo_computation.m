function dist = stereo_computation(leftImage, rightImage, minDisp, maxDisp, winSize, cost_function, weights)

    leftImage = int16(rgb2gray(leftImage));
    rightImage = int16(rgb2gray(rightImage));
    [m,n] = size(leftImage);
    dist = zeros(m,n);
    winHalf = floor(winSize/2);

    for i=1+winHalf:m-winHalf
        for j=1+winHalf:n-winHalf
            bestCost = inf;
            best_win = 0;
            
            winLeft = double(leftImage(i-winHalf:i+winHalf,j-winHalf:j+winHalf));
            for win=minDisp:maxDisp
                if j + win - winHalf <= 0 || j + win + winHalf > n
                   continue; 
                end
                
                winRight = double(rightImage(i-winHalf:i+winHalf,j+win-winHalf:j+win+winHalf));
                w = ones(winSize);
                if (weights==1)
                    for indexQ=1:winSize^2
                        [ip, jp] = ind2sub(winSize^2, indexQ);
                        p = winLeft(winHalf+1,winHalf+1);
                        q = winRight(indexQ);
                        w(indexQ) = exp(double(-(abs(p-q)/12)-(abs(sqrt((i-ip)^2+(j-jp)^2))/17.5)));
                    end
                end

                
                if strcmp('SSD', cost_function)
                    cost = sum(w.*(abs(winLeft(:) - winRight(:))).^2 );
                elseif strcmp('SAD', cost_function)
                    cost = sum(w.*abs(winLeft(:) - winRight(:)));
                elseif strcmp('NCC', cost_function)
                    i1 = winLeft-mean2(w.*winLeft);
                    i2 = winRight-mean2(w.*winRight);
                    corr = sum(sum(w.*i1.*i2))/(sqrt(sum(sum(w.*i1.^2))).*sqrt(sum(sum(w.*i2.^2))));
                    cost = -corr;
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




