%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 2: Image mosaics

addpath('sift');
close all;
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


% ima = sum(double(imargb), 3) / 3 / 255;
% imb = sum(double(imbrgb), 3) / 3 / 255;
% imc = sum(double(imcrgb), 3) / 3 / 255;

ima = rgb2gray(imargb);
imb = rgb2gray(imbrgb);
imc = rgb2gray(imcrgb);

castle_ima = rgb2gray(castle_imargb);
castle_imb = rgb2gray(castle_imbrgb);
castle_imc = rgb2gray(castle_imcrgb);

site13_ima = rgb2gray(site13_imargb);
site13_imb = rgb2gray(site13_imbrgb);
site13_imc = rgb2gray(site13_imcrgb);

% imargb = double(imread('Data/aerial/site22/frame_00001.tif'));
% imbrgb = double(imread('Data/aerial/site22/frame_00018.tif'));
% imcrgb = double(imread('Data/aerial/site22/frame_00030.tif'));
% ima = imargb;
% imb = imbrgb;
% imc = imcrgb;

%% Compute SIFT keypoints
%[points_a, desc_a] = sift(ima, 'Threshold', 0.01);
%[points_b, desc_b] = sift(imb, 'Threshold', 0.01);
%[points_c, desc_c] = sift(imc, 'Threshold', 0.01);

points_a = detectSURFFeatures(ima);
desc_a = extractFeatures(ima, points_a);
points_b = detectSURFFeatures(imb);
desc_b = extractFeatures(imb, points_b);
points_c = detectSURFFeatures(imc);
desc_c = extractFeatures(imc, points_c);



figure;
imshow(imargb);%image(imargb)
hold on;
plot(points_a.Location(:,1), points_a.Location(:,2),'+y');
figure;
imshow(imbrgb);%image(imbrgb);
hold on;
plot(points_b.Location(:,1), points_b.Location(:,2),'+y');
figure;
imshow(imcrgb);%image(imcrgb);
hold on;
plot(points_c.Location(:,1), points_c.Location(:,2),'+y');

%% Match SIFT keypoints 

% between a and b
% matches_ab = siftmatch(desc_a, desc_b);
matches_ab = matchFeatures(desc_a, desc_b);
figure;
plotmatches(ima, imb, points_a.Location', points_b.Location', matches_ab', 'Stacking', 'v');

% between b and c
% matches_bc = siftmatch(desc_b, desc_c);
matches_bc = matchFeatures(desc_b, desc_c);
figure;
plotmatches(imb, imc, points_b.Location', points_c.Location', matches_bc', 'Stacking', 'v');
                
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
title('Mosaic A-B-C');

% ToDo: compute the mosaic with castle_int images

%% 3.2

points_a = detectSURFFeatures(castle_ima);
desc_a = extractFeatures(castle_ima, points_a);
points_b = detectSURFFeatures(castle_imb);
desc_b = extractFeatures(castle_imb, points_b);
points_c = detectSURFFeatures(castle_imc);
desc_c = extractFeatures(castle_imc, points_c);

matches_ab = matchFeatures(desc_a, desc_b);
matches_bc = matchFeatures(desc_b, desc_c);

th = 15;
xab_a = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
xab_b = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];
[Hab, ~] = ransac_homography_adaptive_loop(xab_a, xab_b, th, 1000); % ToDo: complete this function

xbc_b = [points_b.Location(matches_bc(:, 1), :)'; ones(1, length(matches_bc))];
xbc_c = [points_c.Location(matches_bc(:, 2), :)'; ones(1, length(matches_bc))];
[Hbc, ~] = ransac_homography_adaptive_loop(xbc_b, xbc_c, th, 1000); 

corners = [-400 1200 -100 650];
iwb = apply_H_v2(castle_imbrgb, eye(3), corners);   % ToDo: complete the call to the function
iwa = apply_H_v2(castle_imargb, Hab, corners);    % ToDo: complete the call to the function
iwc = apply_H_v2(castle_imcrgb, inv(Hbc), corners);    % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Castle Mosaic A-B-C');

%% 3.3
% ToDo: compute the mosaic with aerial images set 13

points_a = detectSURFFeatures(site13_ima);
desc_a = extractFeatures(site13_ima, points_a);
points_b = detectSURFFeatures(site13_imb);
desc_b = extractFeatures(site13_imb, points_b);
points_c = detectSURFFeatures(site13_imc);
desc_c = extractFeatures(site13_imc, points_c);

matches_ab = matchFeatures(desc_a, desc_b);
matches_bc = matchFeatures(desc_b, desc_c);

th = 10;
xab_a = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
xab_b = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];
[Hab, ~] = ransac_homography_adaptive_loop(xab_a, xab_b, th, 5000); % ToDo: complete this function

xbc_b = [points_b.Location(matches_bc(:, 1), :)'; ones(1, length(matches_bc))];
xbc_c = [points_c.Location(matches_bc(:, 2), :)'; ones(1, length(matches_bc))];
[Hbc, ~] = ransac_homography_adaptive_loop(xbc_b, xbc_c, th, 5000); 

corners = [-400 1200 -100 650];
iwb = apply_H_v2(site13_imbrgb, eye(3), corners);   % ToDo: complete the call to the function
iwa = apply_H_v2(site13_imargb, Hab, corners);    % ToDo: complete the call to the function
iwc = apply_H_v2(site13_imcrgb, inv(Hbc), corners);    % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Aerial 13 Mosaic A-B-C');

%% 3.4
% ToDo: compute the mosaic with aerial images set 22

points_a = detectSURFFeatures(site22_ima);
desc_a = extractFeatures(site22_ima, points_a);
points_b = detectSURFFeatures(site22_imb);
desc_b = extractFeatures(site22_imb, points_b);
points_c = detectSURFFeatures(site22_imc);
desc_c = extractFeatures(site22_imc, points_c);

matches_ab = matchFeatures(desc_a, desc_b);
matches_bc = matchFeatures(desc_b, desc_c);

th = 3;
xab_a = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
xab_b = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];
[Hab, inliers_ab] = ransac_homography_adaptive_loop(xab_a, xab_b, th, 1000); % ToDo: complete this function

xbc_b = [points_b.Location(matches_bc(:, 1), :)'; ones(1, length(matches_bc))];
xbc_c = [points_c.Location(matches_bc(:, 2), :)'; ones(1, length(matches_bc))];
[Hbc, inliers_bc] = ransac_homography_adaptive_loop(xbc_b, xbc_c, th, 1000); 

corners = [-400 1200 -100 650];
iwb = apply_H_v2(site22_imb, eye(3), corners);   % ToDo: complete the call to the function
iwa = apply_H_v2(site22_ima, Hab, corners);    % ToDo: complete the call to the function
iwc = apply_H_v2(site22_imc, inv(Hbc), corners);    % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Aerial 22 Mosaic A-B-C');

% ToDo: comment the results in every of the four cases: say why it works or
%       does not work

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. Refine the homography with the Gold Standard algorithm

% Homography ab

% Assuming normalization
x = points_b.Location(matches_ab(:, 2), :)'; %      point correspondences we will refine with the geometric method
xp =  points_a.Location(matches_ab(:, 1), :)';  %ToDo: set the non-homogeneous point coordinates of the 

P0 = [Hab(:) ; x(:)]; % The parameters or independent variables
P0 = double(P0);

Y_initial = gs_errfunction( P0, x, xp ); % ToDo: create this function that we need to pass to the lsqnonlin function
% NOTE: gs_errfunction should return E(X) and not the sum-of-squares E=sum(E(X).^2)) that we want to minimize. 
% (E(X) is summed and squared implicitly in the lsqnonlin algorithm.) 
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

figure;
imshow(imbrgb); %image(imbrgb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');

%%  Homography bc

% Assuming normalization
x = points_c.Location(matches_bc(:, 2), :)'; %      point correspondences we will refine with the geometric method
xp =  points_b.Location(matches_bc(:, 1), :)';  %ToDo: set the non-homogeneous point coordinates of the 

P0 = [Hbc(:) ; x(:)]; % The parameters or independent variables
P0 = double(P0);

Y_initial = gs_errfunction( P0, x, xp ); % ToDo: create this function that we need to pass to the lsqnonlin function
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

figure;
imshow(imcrgb);%image(imcrgb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');

%% Build mosaic
corners = [-400 1200 -100 650];
iwb = apply_H_v2(imbrgb, eye(3), corners); % ToDo: complete the call to the function
iwa = apply_H_v2(imargb, Hab_r, corners); % ToDo: complete the call to the function
iwc = apply_H_v2(imcrgb, inv(Hbc_r), corners); % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Mosaic A-B-C');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 5. OPTIONAL: Calibration with a planar pattern
% 
% clear all;
% 
% %% Read template and images.
% T     = imread('Data/calib/template.jpg');
% I{1}  = imread('Data/calib/graffiti1.tif');
% I{2}  = imread('Data/calib/graffiti2.tif');
% I{3}  = imread('Data/calib/graffiti3.tif');
% %I{4}  = imread('Data/calib/graffiti4.tif');
% %I{5}  = imread('Data/calib/graffiti5.tif');
% Tg = sum(double(T), 3) / 3 / 255;
% Ig{1} = sum(double(I{1}), 3) / 3 / 255;
% Ig{2} = sum(double(I{2}), 3) / 3 / 255;
% Ig{3} = sum(double(I{3}), 3) / 3 / 255;
% 
% N = length(I);
% 
% %% Compute keypoints.
% fprintf('Computing sift points in template... ');
% [pointsT, descrT] = sift(Tg, 'Threshold', 0.05);
% fprintf(' done\n');
% 
% points = cell(N,1);
% descr = cell(N,1);
% for i = 1:N
%     fprintf('Computing sift points in image %d... ', i);
%     [points{i}, descr{i}] = sift(Ig{i}, 'Threshold', 0.05);
%     fprintf(' done\n');
% end
% 
% %% Match and compute homographies.
% H = cell(N,1);
% for i = 1:N
%     % Match against template descriptors.
%     fprintf('Matching image %d... ', i);
%     matches = siftmatch(descrT, descr{i});
%     fprintf('done\n');
% 
%     % Fit homography and remove outliers.
%     x1 = pointsT(1:2, matches(1, :));
%     x2 = points{i}(1:2, matches(2, :));
%     H{i} = 0;
%     [H{i}, inliers] =  ransac_homography_adaptive_loop(homog(x1), homog(x2), 3, 1000);
% 
%     % Plot inliers.
%     figure;
%     plotmatches(Tg, Ig{i}, pointsT(1:2,:), points{i}(1:2,:), matches(:, inliers));
% 
%     % Play with the homography
%     %vgg_gui_H(T, I{i}, H{i});
% end
% 
% %% Compute the Image of the Absolute Conic
% 
% w = ... % ToDo
%  
% %% Recover the camera calibration.
% 
% K = ... % ToDo
%     
% % ToDo: in the report make some comments related to the obtained internal
% %       camera parameters and also comment their relation to the image size
% 
% %% Compute camera position and orientation.
% R = cell(N,1);
% t = cell(N,1);
% P = cell(N,1);
% figure;hold;
% for i = 1:N
%     % ToDo: compute r1, r2, and t{i}
%     r1 = ...
%     r2 = ...
%     t{i} = ...
%     
%     % Solve the scale ambiguity by forcing r1 and r2 to be unit vectors.
%     s = sqrt(norm(r1) * norm(r2)) * sign(t{i}(3));
%     r1 = r1 / s;
%     r2 = r2 / s;
%     t{i} = t{i} / s;
%     R{i} = [r1, r2, cross(r1,r2)];
%     
%     % Ensure R is a rotation matrix
%     [U S V] = svd(R{i});
%     R{i} = U * eye(3) * V';
%    
%     P{i} = K * [R{i} t{i}];
%     plot_camera(P{i}, 800, 600, 200);
% end
% 
% % ToDo: in the report explain how the optical center is computed in the
% %       provided code
% 
% [ny,nx] = size(T);
% p1 = [0 0 0]';
% p2 = [nx 0 0]';
% p3 = [nx ny 0]';
% p4 = [0 ny 0]';
% % Draw planar pattern
% vgg_scatter_plot([p1 p2 p3 p4 p1], 'g');
% % Paint image texture
% surface('XData',[0 nx; 0 nx],'YData',[0 0; 0 0],'ZData',[0 0; -ny -ny],'CData',T,'FaceColor','texturemap');
% colormap(gray);
% axis equal;
% 
% %% Plot a static camera with moving calibration pattern.
% figure; hold;
% plot_camera(K * eye(3,4), 800, 600, 200);
% % ToDo: complete the call to the following function with the proper
% %       coordinates of the image corners in the new reference system
% for i = 1:N
%     vgg_scatter_plot( [...   ...   ...   ...   ...], 'r');
% end
% 
% %% Augmented reality: Plot some 3D points on every camera.
% [Th, Tw] = size(Tg);
% cube = [0 0 0; 1 0 0; 1 0 0; 1 1 0; 1 1 0; 0 1 0; 0 1 0; 0 0 0; 0 0 1; 1 0 1; 1 0 1; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 0 1; 0 0 0; 1 0 0; 1 0 0; 1 0 1; 1 0 1; 0 0 1; 0 0 1; 0 0 0; 0 1 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 1 0; 0 0 0; 0 1 0; 0 1 0; 0 1 1; 0 1 1; 0 0 1; 0 0 1; 0 0 0; 1 0 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 1 0 1; 1 0 1; 1 0 0 ]';
% 
% X = (cube - .5) * Tw / 4 + repmat([Tw / 2; Th / 2; -Tw / 8], 1, length(cube));
% 
% for i = 1:N
%     figure; colormap(gray);
%     imagesc(Ig{i});
%     hold on;
%     x = euclid(P{i} * homog(X));
%     vgg_scatter_plot(x, 'g');
% end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6. OPTIONAL: Detect the UPF logo in the two UPF images using the 
%%              DLT algorithm (folder "logos").
%%              Interpret and comment the results.

option = "Auto";
if (option =="Manual")
    filenames = {'Data/logos/logo_master.png';'Data/logos/UPFbuilding.jpg'};
    images = cell(size(filenames,1),1);   

    for i=1:size(filenames,1)
        images{i,1}=(imread(filenames{i}));
    end

    % Manually Keypoints Detection
    % ld = 1 load saved points, ld = 0 get new points 
    ld = 1;
    np = 4;     % Number of points to pick.
    [points] = interest_points(images, ld, np);
    img_dst = images{2,1};
    img_src = images{1,1};

    [h,w,c] = size(img_src);

    pts_dst = points(5:6,:);
    pts_src = [0,w-1,w-1,0;0,0,h-1,h-1];
else
    filenames = {'Data/logos/UPFstand.jpg';'Data/logos/logoUPF.png';'Data/logos/logo_master.png'};
    images = cell(size(filenames,1),1);   

    for i=1:size(filenames,1)
        images{i,1}=(imread(filenames{i}));
    end
    
    % Auto Keypoints Detection
    img_dst = images{1,1};
    img_match = images{2,1};
    img_src = images{3,1};
    [rows, cols, a] = size(img_match);
    img_src =  imresize(img_src, [rows cols]);
    
    img_dest_gray = rgb2gray(img_dst);
    img_match_gray = rgb2gray(img_match);
    
    points_a = detectSURFFeatures(img_match_gray);
    desc_a = extractFeatures(img_match_gray, points_a);
    points_b = detectSURFFeatures(img_dest_gray);
    desc_b = extractFeatures(img_dest_gray, points_b);
    
    matches_ab = matchFeatures(desc_a, desc_b);
    
    pts_src = [points_a.Location(matches_ab(:, 1), :)'; ones(1, length(matches_ab))];
    pts_dst = [points_b.Location(matches_ab(:, 2), :)'; ones(1, length(matches_ab))];
        
end

% figure;
% imagesc(img_dst)
% hold on
% plot(pts_dst(1,2), pts_dst(2,2), 'r*')
% hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7. OPTIONAL: Replace the logo of the UPF by the master logo
%%              in one of the previous images using the DLT algorithm.

H = homography2d(pts_src, pts_dst);
[h,w,c] = size(img_dst);

corners = [0 w-1 0 h-1];
[img_transf] = apply_H_v2(img_src , H, corners);
figure
imagesc(img_transf)
figure;
imshow(max(img_transf, img_dst));

