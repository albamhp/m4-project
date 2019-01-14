%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 3: The geometry of two views 
% (application: photo-sequencing)

addpath('../lab2/sift');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Compute the fundamental matrix

% Two camera matrices for testing purposes
P1 = eye(3,4);
c = cosd(15); s = sind(15);
R = [c -s 0; s c 0; 0 0 1];
t = [.3 0.1 0.2]';
P2 = [R t];
n = 8;
X = [rand(3,n); ones(1,n)] + [zeros(2,n); 3 * ones(1,n); zeros(1,n)];
x1_test = P1 * X;
x2_test = P2 * X;

x1_test = x1_test ./ repmat(x1_test(end,:), size(x1_test,1), 1);
x2_test = x2_test ./ repmat(x2_test(end,:), size(x2_test,1), 1);

% Estimated fundamental matrix
F_es = normalized_fundamental_matrix(x1_test, x2_test);

% Real fundamental matrix
tx = [0 -t(3) t(2) ; t(3) 0 -t(1) ; -t(2) t(1) 0];
E = tx * R;
F_gt = E;

% Evaluation: these two matrices should be very similar
A1 = F_gt / norm(F_gt);
A2 = F_es / norm(F_es);

%since obained matrix is up to scale and can be multiplied by -1
if (sign(A1(1,1)) ~= sign(A2(1,1)))
    A2 = -A2;
end
    
disp("Difference between estimated and real fundamental matrices: ")
disp(A1-A2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Robustly fit fundamental matrix

% Read images
im1rgb = imread('Data/0000_s.png');
im2rgb = imread('Data/0001_s.png');
im1 = sum(double(im1rgb), 3) / 3 / 255;
im2 = sum(double(im2rgb), 3) / 3 / 255;

% show images
figure;
subplot(1,2,1); imshow(im1rgb); axis image; title('Image 1');
subplot(1,2,2); imshow(im2rgb); axis image; title('Image 2');


%% Compute SIFT keypoints

% (make sure that the sift folder provided in lab2 is on the path)

[points_1, desc_1] = sift(im1, 'Threshold', 0.01);
[points_2, desc_2] = sift(im2, 'Threshold', 0.01);

%% Match SIFT keypoints between a and b
matches = siftmatch(desc_1, desc_2);
figure;
plotmatches(im1, im2, points_1(1:2,:), points_2(1:2,:), matches, 'Stacking', 'v');

% p1 and p2 contain the homogeneous coordinates of the matches
p1 = [points_1(1:2, matches(1,:)); ones(1, length(matches))];
p2 = [points_2(1:2, matches(2,:)); ones(1, length(matches))];

% If using @algebraic_error, choose 0.005 as threshold
[F, inliers] = ransac_fundamental_matrix(p1, p2, @geometric_error, 2, 1000); 

% show inliers
figure;
plotmatches(im1, im2, points_1(1:2,:), points_2(1:2,:), matches(:,inliers), 'Stacking', 'v');
title('Inliers');

vgg_gui_F(im1rgb, im2rgb, F');


%% Plot some epipolar lines

l2 = F * p1; % epipolar lines in image 2
l1 = F' * p2; % epipolar lines in image 1

% choose three random indices
m1 = inliers(10);
m2 = inliers(20);
m3 = inliers(30);

% image 1 (plot the three points and their corresponding epipolar lines)
figure;
imshow(im1rgb);
hold on;
plot(p1(1, m1), p1(2, m1), '+g');
plot_homog_line(l1(:, m1));

plot(p1(1, m2), p1(2, m2), '+g');
plot_homog_line(l1(:, m2));

plot(p1(1, m3), p1(2, m3), '+g');
plot_homog_line(l1(:, m3));

% image 2 (plot the three points and their corresponding epipolar lines)
figure;
imshow(im2rgb);
hold on;
plot(p2(1, m1), p2(2, m1), '+g');
plot_homog_line(l2(:, m1));

plot(p2(1, m2), p2(2, m2), '+g');
plot_homog_line(l2(:, m2));

plot(p2(1, m3), p2(2, m3), '+g');
plot_homog_line(l2(:, m3));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Photo-sequencing with aerial images

% In this part we will compute a simplified version of the algorithm
% explained in the Photo-sequencing paper. 
% Since we do not have two images
% taken from roughly the same viewpoint at two different time instants we
% will manually pick a dynamic point corresponding to a point in a van 
% (identified by index 'idx_car_I1') and the projection of its 3D trajectory 
% in the reference image. Then we will compute the projection (to the reference image) 
% of three points on this 3D trajectory at three different time instants 
% (corresponding to the time when the three other provided images where taken). 

% Read images
im1rgb = imread('Data/frame_00000.tif');
im2rgb = imread('Data/frame_00001.tif');
im3rgb = imread('Data/frame_00002.tif');
im4rgb = imread('Data/frame_00003.tif');

im1 = sum(double(im1rgb), 3) / 3 / 255;
im2 = sum(double(im2rgb), 3) / 3 / 255;
im3 = sum(double(im3rgb), 3) / 3 / 255;
im4 = sum(double(im4rgb), 3) / 3 / 255;

% show images
figure;
subplot(2,2,1); imshow(im1rgb); axis image; title('Image 1');
subplot(2,2,2); imshow(im2rgb); axis image; title('Image 2');
subplot(2,2,3); imshow(im3rgb); axis image; title('Image 3');
subplot(2,2,4); imshow(im4rgb); axis image; title('Image 4');

% Compute SIFT keypoints
[points_1, desc_1] = sift(im1, 'Threshold', 0.015); % Do not change this threshold!
[points_2, desc_2] = sift(im2, 'Threshold', 0.015);
[points_3, desc_3] = sift(im3, 'Threshold', 0.015);
[points_4, desc_4] = sift(im4, 'Threshold', 0.015);

points_1 = [points_1(1:2, :); ones(1, length(points_1))];
points_2 = [points_2(1:2, :); ones(1, length(points_2))];
points_3 = [points_3(1:2, :); ones(1, length(points_3))];
points_4 = [points_4(1:2, :); ones(1, length(points_4))];

% Calculate all the fundamental matrices with respect to the first image
matches_1_2 = siftmatch(desc_1, desc_2);
matches_1_3 = siftmatch(desc_1, desc_3);
matches_1_4 = siftmatch(desc_1, desc_4);

[F_1_2, inliers_1_2] = ransac_fundamental_matrix(points_1(:, matches_1_2(1, :)), ...
                                                 points_2(:, matches_1_2(2, :)), ...
                                                 @geometric_error, 2, 1000); 

[F_1_3, inliers_1_3] = ransac_fundamental_matrix(points_1(:, matches_1_3(1, :)), ...
                                                 points_3(:, matches_1_3(2, :)), ...
                                                 @geometric_error, 2, 1000); 

[F_1_4, inliers_1_4] = ransac_fundamental_matrix(points_1(:, matches_1_4(1, :)), ...
                                                 points_4(:, matches_1_4(2, :)), ...
                                                 @geometric_error, 2, 1000); 


%% Plot the car trajectory (keypoint idx_car_I1 in image 1)

% Obtain the point indices representing the van
idx_car_I1 = 1197; % given data
idx_car_I2 = matches_1_2(2, matches_1_2(1, :) == idx_car_I1);
idx_car_I3 = matches_1_3(2, matches_1_3(1, :) == idx_car_I1);
idx_car_I4 = matches_1_4(2, matches_1_4(1, :) == idx_car_I1);

% Calculate the trajectory of the van
point1_1 = [points_1(1:2,idx_car_I1); 1];
point1_2 = [334 697 1]'; % given data
l1 = cross(point1_1, point1_2);

% Plot the line
figure;imshow(im1);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(points_1(1,1197), points_1(2,1197), 'y*');

% Calculate the epipolar line
point2 = points_2(1:3, idx_car_I2);
l2 = F_1_2' * point2;
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'c');

% Calculate the projection in the first image
pi2 = cross(l1, l2);
plot(pi2(1)/pi2(3), pi2(2)/pi2(3), 'c*');

% Calculate the epipolar line
point3 = points_3(1:3, idx_car_I3);
l3 = F_1_3' * point3;
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'b');

% Calculate the projection in the first image
pi3 = cross(l1, l3);
plot(pi3(1)/pi3(3), pi3(2)/pi3(3), 'b*');

% Calculate the epipolar line
point4 = points_4(1:3, idx_car_I4);
l4 = F_1_4' * point4;
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'g');

% Calculate the projection in the first image
pi4 = cross(l1, l4);
plot(pi4(1)/pi4(3), pi4(2)/pi4(3), 'g*');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. OPTIONAL: Photo-sequencing with your own images

% 4.1 Take a set of images of a moving scene from different viewpoints at 
%     different time instants. At least two images have to be taken from
%     roughly the same location by the same camera.
%
% 4.2 Implement the first part (until line 16) of the Algorithm 1 of the 
%     Photo-sequencing paper with a selection of the detected dynamic
%     features. You may reuse the code generated for the previous question.
%

base_rgb = imread('Data/SkateBoard/IMG0_002.png');
reference_rgb = imread('Data/SkateBoard/IMG0_009.png');

n_frames = 2;
frames_rgb = cell(1, n_frames);
frames_rgb{1} = imread('Data/SkateBoard/IMG0_003.png');
frames_rgb{2} = imread('Data/SkateBoard/IMG0_004.png');

base = sum(double(base_rgb), 3) / 3 / 255;
reference = sum(double(reference_rgb), 3) / 3 / 255;

frames = cell(1, n_frames);
for i=1:n_frames
    frames{i} = sum(double(frames_rgb{i}), 3) / 3 / 255;
end

% show images
figure;
subplot(1,2,1); imshow(base_rgb); axis image; title('Base image');
subplot(1,2,2); imshow(reference_rgb); axis image; title('Reference image');

% Compute SIFT keypoints
[points_base, desc_base] = sift(base, 'Threshold', 0.015); % Do not change this threshold!
points_base = [points_base(1:2, :); ones(1, length(points_base))];

[points_reference, desc_reference] = sift(reference, 'Threshold', 0.015);
points_reference = [points_reference(1:2, :); ones(1, length(points_reference))];

keypoints = cell(2, n_frames);
for i=1:n_frames
    [keypoints{1, i}, keypoints{2, i}] = sift(frames{i}, 'Threshold', 0.015); % Do not change this threshold!
    keypoints{1, i} = [keypoints{1, i}(1:2, :); ones(1, length(keypoints{1, i}))];
end

matches_base_reference = siftmatch(desc_base, desc_reference);

matches = cell(1, n_frames);
for i=1:n_frames
    matches{i} = siftmatch(desc_base, keypoints{2, i});
end

fundamental_matrices = cell(2, n_frames);
for i=1:n_frames
    [fundamental_matrices{1, i}, fundamental_matrices{2, i}] = ...
            ransac_fundamental_matrix(points_base(:, matches{i}(1, :)), ...
                                      keypoints{1, i}(:, matches{i}(2, :)), ...
                                      @geometric_error, 2, 2000); 
end
                                             
% Obtain the point indices representing the van
indices = zeros(1 + n_frames, length(points_base));

idx_skate_base = 1:10; % given data
indices(1, :) = ismember(1:length(points_base), matches_base_reference(1, :));

for i=1:n_frames
    indices(1+i, :) = ismember(1:length(points_base), matches{i}(1, :));
end

shared_indices = find(sum(indices) == 1 + n_frames);

% 142 is good
idx_base = 142; % given data
idx_reference = matches_base_reference(2, matches_base_reference(1, :) == idx_base);

idx = cell(1, n_frames);
for i=1:n_frames
    idx{i} = matches{i}(2, matches{i}(1, :) == idx_base);
end

% Calculate the trajectory of the van
point1_1 = points_base(:, idx_base);
point1_2 = points_reference(:, idx_reference);
l1 = cross(point1_1, point1_2);

% Plot the line
figure;imshow(base);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(point1_1(1), point1_1(2), 'y*');

colors = ['c', 'b', 'g'];
for i=1:n_frames
    % Calculate the epipolar line
    point = keypoints{1, i}(1:3, idx{i});
    l = fundamental_matrices{1, i}' * point;
    plot(t, -(l(1)*t + l(3)) / l(2), colors(i));

    % Calculate the projection in the first image
    pi = cross(l1, l);
    plot(pi(1)/pi(3), pi(2)/pi(3), strcat(colors(i), '*'));
end
