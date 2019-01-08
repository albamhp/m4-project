%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 2: Image mosaics

close all;
clear all;
clc

addpath sift;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Compute image correspondences

%% Open images

imargb = imread('Data/llanes/llanes_a.jpg');
imbrgb = imread('Data/llanes/llanes_b.jpg');
imcrgb = imread('Data/llanes/llanes_c.jpg');

castle_imargb = imread('Data/castle_int/0016_s.png');
castle_imbrgb = imread('Data/castle_int/0015_s.png');
castle_imcrgb = imread('Data/castle_int/0014_s.png');

site13_imargb = imread('Data/aerial/site13/frame00000.png');
site13_imbrgb = imread('Data/aerial/site13/frame00002.png');
site13_imcrgb = imread('Data/aerial/site13/frame00003.png');

site22_ima = imread('Data/aerial/site22/frame_00001.tif');
site22_imb = imread('Data/aerial/site22/frame_00018.tif');
site22_imc = imread('Data/aerial/site22/frame_00030.tif');

%% Images to grayscale

ima = rgb2gray(imargb);
imb = rgb2gray(imbrgb);
imc = rgb2gray(imcrgb);

castle_ima = rgb2gray(castle_imargb);
castle_imb = rgb2gray(castle_imbrgb);
castle_imc = rgb2gray(castle_imcrgb);

site13_ima = rgb2gray(site13_imargb);
site13_imb = rgb2gray(site13_imbrgb);
site13_imc = rgb2gray(site13_imcrgb);


%% Compute SURF keypoints

points_a = detectSURFFeatures(ima);
desc_a = extractFeatures(ima, points_a);
points_b = detectSURFFeatures(imb);
desc_b = extractFeatures(imb, points_b);
points_c = detectSURFFeatures(imc);
desc_c = extractFeatures(imc, points_c);

figure;
subplot(1,3,1)
imshow(imargb);
hold on;
plot(points_a.Location(:,1), points_a.Location(:,2),'+y');
hold off;
title('Image a SURF keypoints')
subplot(1,3,2)
imshow(imbrgb);
hold on;
plot(points_b.Location(:,1), points_b.Location(:,2),'+y');
hold off;
title('Image b SURF keypoints')
subplot(1,3,3)
imshow(imcrgb);
hold on;
plot(points_c.Location(:,1), points_c.Location(:,2),'+y');
hold off;
title('Image c SURF keypoints')

%% Match SURF keypoints 

% between a and b
matches_ab = matchFeatures(desc_a, desc_b);
figure;
plotmatches(ima, imb, points_a.Location', points_b.Location',...
    matches_ab', 'Stacking', 'v');

% between b and c
matches_bc = matchFeatures(desc_b, desc_c);
figure
plotmatches(imb, imc, points_b.Location', points_c.Location',...
    matches_bc', 'Stacking', 'v');
                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Compute the homography (DLT algorithm) between image pairs

%% Compute homography (normalized DLT) between a and b, play with the homography
th = 3;
xab_a = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
xab_b = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];

[Hab, inliers_ab] = ransac_homography_adaptive_loop(xab_a, xab_b, th, 5000); % ToDo: complete this function

figure;
plotmatches(ima, imb, points_a.Location', points_b.Location', ...
    matches_ab(inliers_ab, :)', 'Stacking', 'v');

vgg_gui_H(imargb, imbrgb, Hab);


%% Compute homography (normalized DLT) between b and c, play with the homography
xbc_b = [points_b.Location(matches_bc(:, 1), :)'; ones(1, length(matches_bc))];
xbc_c = [points_c.Location(matches_bc(:, 2), :)'; ones(1, length(matches_bc))];

[Hbc, inliers_bc] = ransac_homography_adaptive_loop(xbc_b, xbc_c, th, 5000); 

figure;
plotmatches(imb, imc, points_b.Location', points_c.Location', ...
    matches_bc(inliers_bc, :)', 'Stacking', 'v');

vgg_gui_H(imbrgb, imcrgb, Hbc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Build the mosaic

corners = [-400 1200 -100 650];
iwb = apply_H_v2(imbrgb, eye(3), corners);   % ToDo: complete the call to the function
iwa = apply_H_v2(imargb, Hab, corners);    % ToDo: complete the call to the function
iwc = apply_H_v2(imcrgb, inv(Hbc), corners);    % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Llanes Mosaic A-B-C');

%% 3.2
% ToDo: compute the mosaic with castle_int images

points_a2 = detectSURFFeatures(castle_ima);
desc_a2 = extractFeatures(castle_ima, points_a2);
points_b2 = detectSURFFeatures(castle_imb);
desc_b2 = extractFeatures(castle_imb, points_b2);
points_c2 = detectSURFFeatures(castle_imc);
desc_c2 = extractFeatures(castle_imc, points_c2);

matches_ab2 = matchFeatures(desc_a2, desc_b2);
matches_bc2 = matchFeatures(desc_b2, desc_c2);

th2 = 15;
xab_a2 = [points_a2.Location(matches_ab2(:, 1), :)'; ones(1, length(matches_ab2))];
xab_b2 = [points_b2.Location(matches_ab2(:, 2), :)'; ones(1, length(matches_ab2))];
[Hab2, ~] = ransac_homography_adaptive_loop(xab_a2, xab_b2, th2, 1000); % ToDo: complete this function

xbc_b2 = [points_b2.Location(matches_bc2(:, 1), :)'; ones(1, length(matches_bc2))];
xbc_c2 = [points_c2.Location(matches_bc2(:, 2), :)'; ones(1, length(matches_bc2))];
[Hbc2, ~] = ransac_homography_adaptive_loop(xbc_b2, xbc_c2, th2, 1000); 

corners2 = [-400 1200 -100 650];
iwb2 = apply_H_v2(castle_imbrgb, eye(3), corners2);   % ToDo: complete the call to the function
iwa2 = apply_H_v2(castle_imargb, Hab2, corners2);    % ToDo: complete the call to the function
iwc2 = apply_H_v2(castle_imcrgb, inv(Hbc2), corners2);    % ToDo: complete the call to the function

figure;
imshow(max(iwc2, max(iwb2, iwa2)));
title('Castle Mosaic A-B-C');

%% 3.3
% ToDo: compute the mosaic with aerial images set 13

points_a3 = detectSURFFeatures(site13_ima);
desc_a3 = extractFeatures(site13_ima, points_a3);
points_b3 = detectSURFFeatures(site13_imb);
desc_b3 = extractFeatures(site13_imb, points_b3);
points_c3 = detectSURFFeatures(site13_imc);
desc_c3 = extractFeatures(site13_imc, points_c3);

matches_ab3 = matchFeatures(desc_a3, desc_b3);
matches_bc3 = matchFeatures(desc_b3, desc_c3);

th3 = 10;
xab_a3 = [points_a3.Location(matches_ab3(:, 1), :)'; ones(1, length(matches_ab3))];
xab_b3 = [points_b3.Location(matches_ab3(:, 2), :)'; ones(1, length(matches_ab3))];
[Hab3, ~] = ransac_homography_adaptive_loop(xab_a3, xab_b3, th3, 5000); % ToDo: complete this function

xbc_b3 = [points_b3.Location(matches_bc3(:, 1), :)'; ones(1, length(matches_bc3))];
xbc_c3 = [points_c3.Location(matches_bc3(:, 2), :)'; ones(1, length(matches_bc3))];
[Hbc3, ~] = ransac_homography_adaptive_loop(xbc_b3, xbc_c3, th3, 5000); 

corners3 = [-400 1200 -100 650];
iwb3 = apply_H_v2(site13_imbrgb, eye(3), corners3);   % ToDo: complete the call to the function
iwa3 = apply_H_v2(site13_imargb, Hab3, corners3);    % ToDo: complete the call to the function
iwc3 = apply_H_v2(site13_imcrgb, inv(Hbc3), corners3);    % ToDo: complete the call to the function

figure;
imshow(max(iwc3, max(iwb3, iwa3)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Aerial 13 Mosaic A-B-C');

%% 3.4
% ToDo: compute the mosaic with aerial images set 22

points_a4 = detectSURFFeatures(site22_ima);
desc_a4 = extractFeatures(site22_ima, points_a4);
points_b4 = detectSURFFeatures(site22_imb);
desc_b4 = extractFeatures(site22_imb, points_b4);
points_c4 = detectSURFFeatures(site22_imc);
desc_c4 = extractFeatures(site22_imc, points_c4);

matches_ab4 = matchFeatures(desc_a4, desc_b4);
matches_bc4 = matchFeatures(desc_b4, desc_c4);

th4 = 3;
xab_a4 = [points_a4.Location(matches_ab4(:, 1), :)'; ones(1, length(matches_ab4))];
xab_b4 = [points_b4.Location(matches_ab4(:, 2), :)'; ones(1, length(matches_ab4))];
[Hab4, inliers_ab4] = ransac_homography_adaptive_loop(xab_a4, xab_b4, th4, 1000); % ToDo: complete this function

xbc_b4 = [points_b4.Location(matches_bc4(:, 1), :)'; ones(1, length(matches_bc4))];
xbc_c4 = [points_c4.Location(matches_bc4(:, 2), :)'; ones(1, length(matches_bc4))];
[Hbc4, inliers_bc4] = ransac_homography_adaptive_loop(xbc_b4, xbc_c4, th4, 1000); 

corners4 = [-400 1200 -100 650];
iwb4 = apply_H_v2(site22_imb, eye(3), corners4);   % ToDo: complete the call to the function
iwa4 = apply_H_v2(site22_ima, Hab4, corners4);    % ToDo: complete the call to the function
iwc4 = apply_H_v2(site22_imc, inv(Hbc4), corners4);    % ToDo: complete the call to the function

figure;
imshow(max(iwc4, max(iwb4, iwa4)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Aerial 22 Mosaic A-B-C');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. Refine the homography with the Gold Standard algorithm

%% Homography ab

% Assuming normalization
xp = points_b.Location(matches_ab(inliers_ab, 2), :)'; % point correspondences we will refine with the geometric method
x =  points_a.Location(matches_ab(inliers_ab, 1), :)';  %ToDo: set the non-homogeneous point coordinates of the 

P0 = [Hab(:) ; x(:)]; % The parameters or independent variables
P0 = double(P0);

Y_initial = gs_errfunction( P0, x, xp);
err_initial = sum( sum( Y_initial.^2 ));

options = optimset('Algorithm', 'levenberg-marquardt');
P = lsqnonlin(@(t) gs_errfunction(t, x, xp), P0, [], [], options);

Hab_r = reshape(P(1:9), 3, 3 );
f = gs_errfunction( P, x, xp ); % lsqnonlin does not return f
err_final = sum( sum( f.^2 ));

% we show the geometric error before and after the refinement
fprintf(1, 'Gold standard reproj error initial %f, final %f\n', err_initial, err_final);


%% See differences in the keypoint locations

% ToDo: compute the points xhat and xhatp which are the correspondences
% returned by the refinement with the Gold Standard algorithm
xhat = P(10:end);
xhat = reshape(xhat, 2, length(P0(10:end))/2);
xhat = [xhat; ones(1, length(P0(10:end))/2)];
xhatp = Hab_r * xhat;

figure;
imshow(imargb); %image(imargb);
hold on;
plot(xp(1,:), xp(2,:),'+y');
plot(xhatp(1,:), xhatp(2,:),'+c');
hold off;

figure;
imshow(imbrgb); %image(imbrgb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');
hold off;

%%  Homography bc

% Assuming normalization
xp = points_c.Location(matches_bc(inliers_bc, 2), :)'; %      point correspondences we will refine with the geometric method
x =  points_b.Location(matches_bc(inliers_bc, 1), :)';  %ToDo: set the non-homogeneous point coordinates of the 

P0 = [Hbc(:) ; x(:)]; % The parameters or independent variables
P0 = double(P0);

Y_initial = gs_errfunction( P0, x, xp); % ToDo: create this function that we need to pass to the lsqnonlin function
% NOTE: gs_errfunction should return E(X) and not the sum-of-squares E=sum(E(X).^2)) that we want to minimize. 
% (E(X) is summed and squared implicitly in the lsqnonlin algorithm.) 
err_initial = sum( sum( Y_initial.^2 ));

options = optimset('Algorithm', 'levenberg-marquardt');
P = lsqnonlin(@(t) gs_errfunction(t, x, xp), P0, [], [], options);

Hbc_r = reshape(P(1:9), 3, 3 );
f = gs_errfunction( P, x, xp ); % lsqnonlin does not return f
err_final = sum( sum( f.^2 ));

% we show the geometric error before and after the refinement
fprintf(1, 'Gold standard reproj error initial %f, final %f\n', err_initial, err_final);


%% See differences in the keypoint locations

% ToDo: compute the points xhat and xhatp which are the correspondences
% returned by the refinement with the Gold Standard algorithm
xhat = P(10:end);
xhat = reshape(xhat, 2, length(P0(10:end))/2);
xhat = [xhat; ones(1, length(P0(10:end))/2)];
xhatp = Hab_r * xhat;

figure;
imshow(imbrgb);%image(imbrgb);
hold on;
plot(xp(1,:), xp(2,:),'+y');
plot(xhatp(1,:), xhatp(2,:),'+c');
hold off;

figure;
imshow(imcrgb);%image(imcrgb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');
hold off;

%% Build mosaic
corners = [-400 1200 -100 650];
iwb = apply_H_v2(imbrgb, eye(3), corners); % ToDo: complete the call to the function
iwa = apply_H_v2(imargb, Hab_r, corners); % ToDo: complete the call to the function
iwc = apply_H_v2(imcrgb, inv(Hbc_r), corners); % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Mosaic A-B-C');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5. OPTIONAL: Calibration with a planar pattern

clear all;
close all;

%% Read template and images.
T     = imread('Data/calib/template.jpg');
I{1}  = imread('Data/calib/graffiti1.tif');
I{2}  = imread('Data/calib/graffiti2.tif');
I{3}  = imread('Data/calib/graffiti3.tif');
%I{4}  = imread('Data/calib/graffiti4.tif');
%I{5}  = imread('Data/calib/graffiti5.tif');
Tg = sum(double(T), 3) / 3 / 255;
Ig{1} = sum(double(I{1}), 3) / 3 / 255;
Ig{2} = sum(double(I{2}), 3) / 3 / 255;
Ig{3} = sum(double(I{3}), 3) / 3 / 255;

N = length(I);

%% Compute keypoints.
fprintf('Computing sift points in template... ');
pointsT = detectSURFFeatures(Tg);
descrT = extractFeatures(Tg, pointsT);
fprintf(' done\n');

points = cell(N,1);
descr = cell(N,1);
for i = 1:N
    fprintf('Computing sift points in image %d... ', i);
    points{i} = detectSURFFeatures(Ig{i});
    descr{i} = extractFeatures(Ig{i}, points{i});
    fprintf(' done\n');
end

%% Match and compute homographies.
H = cell(N,1);
for i = 1:N
    % Match against template descriptors.
    fprintf('Matching image %d... ', i);
    matches = matchFeatures(descrT, descr{i});
    fprintf('done\n');

    % Fit homography and remove outliers.
    x1 = [pointsT.Location(matches(:,1), 1:2)'; ones(1, length(matches))];
    x2 = [points{i}.Location(matches(:,2), 1:2)';ones(1, length(matches))];
    H{i} = 0;
    [H{i}, inliers] =  ransac_homography_adaptive_loop(x1, x2, 3, 1000);

    % Plot inliers.
    figure;
    plotmatches(Tg, Ig{i}, pointsT.Location(:, 1:2)', points{i}.Location(:,1:2)', matches(inliers,:)');

    % Play with the homography
    %vgg_gui_H(T, I{i}, H{i});
end

%% Compute the Image of the Absolute Conic

V = zeros(6, 6);
for i = 1:N
    V(2*i - 1, :) = calculate_V(H{i}, 1, 2)'; 
    V(2*i, :) = calculate_V(H{i}, 1, 1)' - calculate_V(H{i}, 2, 2)';
end

[~, ~, V] = svd(V);

omega = V(:, 6);
w = [omega(1), omega(2), omega(3);
     omega(2), omega(4), omega(5); 
     omega(3), omega(5), omega(6)];
 
%% Recover the camera calibration.

K = chol(inv(w));
K = K ./ K(3, 3);
    
% ToDo: in the report make some comments related to the obtained internal
%       camera parameters and also comment their relation to the image size

%% Compute camera position and orientation.
R = cell(N,1);
t = cell(N,1);
P = cell(N,1);
figure;hold;
for i = 1:N
    % ToDo: compute r1, r2, and t{i}

    r1 = K \ H{i}(:, 1);
    r2 = K \ H{i}(:, 2);
    t{i} = K \ H{i}(:, 3);
    
    % Solve the scale ambiguity by forcing r1 and r2 to be unit vectors.
    s = sqrt(norm(r1) * norm(r2)) * sign(t{i}(3));
    r1 = r1 / s;
    r2 = r2 / s;
    t{i} = t{i} / s;
    R{i} = [r1, r2, cross(r1,r2)];
    
    % Ensure R is a rotation matrix
    [U, S, V] = svd(R{i});
    R{i} = U * eye(3) * V';
   
    P{i} = K * [R{i} t{i}];
    plot_camera(P{i}, 800, 600, 200);
end

% ToDo: in the report explain how the optical center is computed in the
%       provided code

[ny,nx] = size(T);
p1 = [-nx/2 -ny/2 0]';
p2 = [nx/2 -ny/2 0]';
p3 = [nx/2 ny/2 0]';
p4 = [-nx/2 ny/2 0]';
% Draw planar pattern
vgg_scatter_plot([p1 p2 p3 p4 p1], 'g');
% Paint image texture
surface('XData',[0 nx/2; 0 nx/2],'YData',[0 0; 0 0],'ZData',[0 0; -ny/2 -ny/2],'CData',T,'FaceColor','texturemap');
colormap(gray);
axis equal;

%% Plot a static camera with moving calibration pattern.
figure; hold;
plot_camera(K * eye(3,4), 800, 600, 200);
% ToDo: complete the call to the following function with the proper
%       coordinates of the image corners in the new reference system
corners = [p1, p2, p3, p4, p1; ones(1, 5)];
for i = 1:N
    vgg_scatter_plot([inv(R{i}), t{i}] * corners, 'r');
end

%% Augmented reality: Plot some 3D points on every camera.
[Th, Tw] = size(Tg);
cube = [0 0 0; 1 0 0; 1 0 0; 1 1 0; 1 1 0; 0 1 0; 0 1 0; 0 0 0; 0 0 1; 1 0 1; 1 0 1; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 0 1; 0 0 0; 1 0 0; 1 0 0; 1 0 1; 1 0 1; 0 0 1; 0 0 1; 0 0 0; 0 1 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 1 0; 0 0 0; 0 1 0; 0 1 0; 0 1 1; 0 1 1; 0 0 1; 0 0 1; 0 0 0; 1 0 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 1 0 1; 1 0 1; 1 0 0 ]';

X = (cube - .5) * Tw / 4 + repmat([Tw / 2; Th / 2; -Tw / 8], 1, length(cube));
X = [X; ones(1, length(X))];


for i = 1:N
    figure; colormap(gray);
    imagesc(Ig{i});
    hold on;
    x = euclid(P{i} * X);
    vgg_scatter_plot(x, 'g');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6. OPTIONAL: Detect the UPF logo in the two UPF images using the 
%%              DLT algorithm (folder "logos").
%%              Interpret and comment the results.

%%
% Manually Keypoints Detection

img_dst = imread('Data/logos/UPFbuilding.jpg');
img_src = imread('Data/logos/logo_master.png');

load points.mat points

[h,w,c] = size(img_src);

pts_dst = points(1:2,:);
pts_src = [0,w-1,w-1,0;0,0,h-1,h-1];
    
% Auto Keypoints Detection

img_dst_auto = imread('Data/logos/UPFstand.jpg');
img_match_auto = imread('Data/logos/logoUPF.png');
img_src_auto = imread('Data/logos/logo_master.png');

[rows, cols, a] = size(img_match_auto);
img_src_auto =  imresize(img_src_auto, [rows cols]);

img_dest_gray = rgb2gray(img_dst_auto);
img_match_gray = rgb2gray(img_match_auto);

points_a = detectSURFFeatures(img_match_gray);
desc_a = extractFeatures(img_match_gray, points_a);
points_b = detectSURFFeatures(img_dest_gray);
desc_b = extractFeatures(img_dest_gray, points_b);

matches_ab = matchFeatures(desc_a, desc_b);

pts_src_auto = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
pts_dst_auto = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7. OPTIONAL: Replace the logo of the UPF by the master logo
%%              in one of the previous images using the DLT algorithm.

% MANUAL
H = homography2d(pts_src, pts_dst);
[h,w,c] = size(img_dst);

corners = [0 w-1 0 h-1];
[img_transf] = apply_H_v2(img_src , H, corners);
figure;
imshow(max(img_transf, img_dst));

% AUTO
H = homography2d(pts_src_auto, pts_dst_auto);
[h,w,c] = size(img_dst_auto);

corners = [0 w-1 0 h-1];
[img_transf_auto] = apply_H_v2(img_src_auto , H, corners);
figure;
imshow(max(img_transf_auto, img_dst_auto));
