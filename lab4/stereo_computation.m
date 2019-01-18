function dist = stereo_computation(leftImage, rightImage, minDisp, maxDisp, winSize, cost)


[m,n,z]=size(leftImage);

leftImage=rgb2gray(leftImage);
rightImage=rgb2gray(rightImage);

c1=0;c2=0;
bestSSD = ones(winSize,winSize)*inf;
dist = zeros(m,n);
winHalf = floor(winSize/2);
if strcmp('SSD', cost)
    for i=1+winHalf:m-winHalf
        for j=1+winHalf:n-winHalf
            winLeft = leftImage(i-winHalf:i+winHalf,j-winHalf:j+winHalf);
            for win=1+winHalf:n-winHalf
                winRight = rightImage(i-winHalf:i+winHalf,win-winHalf:win+winHalf);
                
                SSD = sum((winLeft - winRight)^2);
                
                if SSD < bestSSD
                    hpos = win;
                    bestSSD = SSD;
                elseif SSD == bestSSD
                    hpos = [hpos win];
                end
            end
            [~,minIndex] = min(abs(hpos-j));
            dist(i,j) = hpos(minIndex);
        end    
    end
elseif strcmp('NCC', cost)
    for i=1:m
        for j=maxDisp+1:n
          c1=c1+(leftImage(i,j)-a1);
          c2=c2+(b(i,j-maxDisp)-b1);
          num=c1*c2;
          c3=(c1^2)*(c2^2);
          dem=sqrt(c3);
        end
    end
    ncc=num/dem;
else 
    disp('Wrong cost function')
end

end




