function [points] = interest_points(images, ld, num)
% - images: array of images to match.
% - ld: is a boolean, if the flag is set to 0, then you pick manually the
%   points, if is set to 1 you load them from the workspace.
% - num: is the number of points to pick (min 4).

    points = nan(2, num); 
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
                point = pick_points(images,num); 
                points(i:3+i,:) = point;
            end
           

           
        end
        close(figure(1));
        % http://es.mathworks.com/help/matlab/ref/save.html
        
        save points.mat points
        
    end

end

function [point] = pick_points(im1, num)

    point = zeros(4,num);
    
    figure(1), imshow(uint8(im1)); hold on,
    
    for p=1:num
        figure(1),
        title('Click on points starting by top left and clock-wise ')
        [y1,x1] = ginput(1);
        plot(y1,x1,'c+');
        
        point(1,p) = y1;
        point(2,p) = x1;
        
        
    end
end