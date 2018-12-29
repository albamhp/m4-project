function [points] = interest_points(images, ld, num)
% - images: array of images to match.
% - ld: is a boolean, if the flag is set to 0, then you pick manually the
%   points, if is set to 1 you load them from the workspace.
% - num: is the number of points to pick (min 4).

    [n,~] = size(images);
    points = nan(4*n, num); 
    % The first 2n rows are correspondences with left image, second 2n rows, 
    % are right image correspondences. If left image or right image doesn't
    % exist, fill with NaN

    if ld == 1
        % http://es.mathworks.com/help/matlab/ref/load.html
        
        load points.mat points
    else
        % Select the point pairs in a pair of images
        for i=1:4:size(points,1)
            % Also fill the function pick_points.
            if i ~= size(points,1)-3
                point = pick_points(images{fix(i/4)+1},images{fix(i/4)+2},num); 
                points(2+i:5+i,:) = point;
            end

           
        end
        close(figure(1));
        close(figure(2));
        % http://es.mathworks.com/help/matlab/ref/save.html
        
        save points.mat points
        
    end

end

function [point] = pick_points(im1, im2, num)

    point = zeros(4,num);
    
    figure(1), imshow(uint8(im1)); hold on,
    figure(2), imshow(uint8(im2)); hold on,
    
    for p=1:num
        figure(1),
        [y1,x1] = ginput(1);
        plot(y1,x1,'c+');
        
        figure(2),
        [y2,x2] = ginput(1);
        plot(y2,x2,'y+');
        
         point(1,p) = y1;
         point(2,p) = x1;
         point(3,p) = y2;
         point(4,p) = x2;
        
        
    end
end