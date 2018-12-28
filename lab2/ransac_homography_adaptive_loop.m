function [H, idx_inliers] = ransac_homography_adaptive_loop(x1, x2, th, max_it)

[Npoints, Ncoords] = size(x1);

% ransac
it = 0;
best_inliers = [];
while it < max_it
    
    points = randomsample(Npoints, 4);
    H = homography2d(x1(points,:)', x2(points,:)'); % ToDo: you have to create this function
    inliers = compute_inliers(H, x1, x2, th);
    
    % test if it is the best model so far
    if length(inliers) > length(best_inliers)
        best_inliers = inliers;
    end    
    
    % update estimate of max_it (the number of trials) to ensure we pick, 
    % with probability p, an initial data set with no outliers
    fracinliers =  length(inliers)/Npoints;
    pNoOutliers = 1 -  fracinliers^4;
    pNoOutliers = max(eps, pNoOutliers);  % avoid division by -Inf
    pNoOutliers = min(1-eps, pNoOutliers);% avoid division by 0
    p=0.99;
    max_it = log(1-p)/log(pNoOutliers);
    
    it = it + 1;
end

% compute H from all the inliers
H = homography2d(x1(:,best_inliers), x2(:,best_inliers));
idx_inliers = best_inliers;


function idx_inliers = compute_inliers(H, x1, x2, th)
    % Check that H is invertible
    if abs(log(cond(H))) > 15
        idx_inliers = [];
        return
    end

    % compute the symmetric geometric error
    x1t = zeros(length(x1),3);
    x2t = zeros(length(x2),3);
    
    for i = 1:length(x1)
        x1t(i,:) = (H*x1(i,:)')';
        x2t(i,:) = (inv(H)*x2(i,:)')';
    end
    
    d2 = computed2(x1t, x2) + computed2(x1, x2t);
    idx_inliers = find(d2 < th.^2);

function d2=computed2(a, b)
    d2 = ((a(:,1)./a(:,3))-(b(:,1)./b(:,3))).^2 ...
    + ((a(:,2)./a(:,3))-(b(:,2)./b(:,3))).^2;


function xn = normalise(x)    
    xn = x ./ repmat(x(end,:), size(x,1), 1);

    
function item = randomsample(npts, n)
	a = [1:npts]; 
    item = zeros(1,n);    
    for i = 1:n
	  % Generate random value in the appropriate range 
	  r = ceil((npts-i+1).*rand);
	  item(i) = a(r);       % Select the rth element from the list
	  a(r)    = a(end-i+1); % Overwrite selected element
    end                       % ... and repeat