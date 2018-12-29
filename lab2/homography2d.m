
function [H] = homography2d(pt1, pt2)
% The Direct Linear Transform (DLT) algorithm is a simple algorithm used
% to solve for the homography matrix H given a sufficient set of point 
% correspondences.

% Number of points
n = length(pt1);

homo=ones(1,n);

if (size(pt1,1) ~= 3)
    pt1 = [pt1;homo];
    pt2 = [pt2;homo];
end

   
% Attempt to normalise each set of points so that the origin 
% is at centroid and mean distance from origin is sqrt(2).
[pt1, T1] = normalise2dpts(pt1);
[pt2, T2] = normalise2dpts(pt2);

x1 = pt1(1,:);
y1 = pt1(2,:);
w1 = pt1(3,:);


x2 = pt2(1,:);
y2 = pt2(2,:);
w2 = pt2(3,:);

% A initialization
A = zeros(8,9);

% Fill A with the proper values at each point
for i=1:n
    A(2*i-1,1:3) = [0,0,0];
    A(2*i-1,4:6) = -w2(i).*[x1(i),y1(i),w1(i)];
    A(2*i-1,7:9) =  y2(i).*[x1(i),y1(i),w1(i)];
    A(2*i,1:3) = -w2(i).*[x1(i),y1(i),w1(i)];
    A(2*i,4:6) =  [0,0,0];
    A(2*i,7:9) = x2(i).*[x1(i),y1(i),w1(i)];
end

% for i=1:n
%     A(i,:) = A(i,:)/norm(A(i,:));
% end

% Singular Value Decomposition
% http://es.mathworks.com/help/matlab/ref/svd.html?s_tid=srchtitle
[~,~,V] =svd(A);

% Take the last column of the transposed of V, that's the singular
% vector with the lowest singular value.
h = V(:,9);

% Reshape h to be a 3x3 matrix.
H = reshape(h,3,3);

% Desnormalizar
H = (T2\H*T1);

H = H ./ H(3, 3);

end