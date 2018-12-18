close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 1: Image rectification


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Applying image transformations

% ToDo: create the function  'apply_H' that gets as input a homography and
% an image and returns the image transformed by the homography.
% The size of the transformed image has to be automatically set so as to 
% contain the whole transformed image.
% At some point you will need to interpolate the image values at some points,
% you may use the Matlab function 'interp2' for that.


%% 1.1. Similarities
I=imread('Data/0005_s.png'); % we have to be in the proper folder

% ToDo: generate a matrix H which produces a similarity transformation

%%% X THE ANGLE OF ROTATION IN THE NEW COORDINATES SYSTEM (NO IDEA...)

theta = 50;
scale = 0.7;
tx = 100;
ty = 100;
T = [1,0,tx; 0,1,ty; 0,0,1];
R = [scale*cos(theta),-scale*sin(theta),0; scale*sin(theta),scale*cos(theta),0; 0,0,1];
H = T*R;


I2 = apply_H(I, H);
figure; imshow(I); figure; imshow(uint8(I2));


%% 1.2. Affinities

% ToDo: generate a matrix H which produces an affine transformation
theta = 0;
phi = 0;
scale1 = 0.7;
scale2 = 0.7;
shx = 0;
shy = -1;


R_theta = [cos(theta),-sin(theta),0; sin(theta),cos(theta),0; 0,0,1];
R_phi_p = [cos(phi),-sin(phi),0; sin(phi),cos(phi),0; 0,0,1];
R_phi_n = [cos(-phi),-sin(-phi),0; sin(-phi),cos(-phi),0; 0,0,1];
S = [scale1,0,0; 0,scale2,0; 0,0,1];
T = [1,0,tx; 0,1,ty; 0,0,1];
SH = [1 shy 0; shx 1 0; 0 0 1];
H1 = T*R_phi_p*S*R_phi_n*R_theta*SH;
I3 = apply_H(I, H1);
figure; imshow(I); figure; imshow(uint8(I3));

% ToDo: decompose the affinity in four transformations: two
% rotations, a scale, and a translation

% ToDo: verify that the product of the four previous transformations
% produces the same matrix H as above
if ( isequal(H,H1))
    disp('The matrix are equal')
else 
    disp('They are not the same')
end

       
% ToDo: verify that the proper sequence of the four previous
% transformations over the image I produces the same image I2 as before
if ( isequal(I2,I3))
    disp('The images are equal')
else
    disp('The images are not equal')
end


%% 1.3 Projective transformations (homographies)

% ToDo: generate a matrix H which produces a projective transformation
H2 = H1;
H2(3,1:2) = [0,0.0005];
I2 = apply_H(I, H2);
figure; imshow(I); figure; imshow(uint8(I2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Affine Rectification
close all

% choose the image points
I=imread('Data/0000_s.png');
A = load('Data/0000_s_info_lines.txt');

% indices of lines

[p1, p2] = get_points (227, A);
[p3, p4] = get_points (367, A);
[p5, p6] = get_points (712, A);
[p7, p8] = get_points (565, A);

% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points

l1 = get_line(p1, p2);
l2 = get_line(p3, p4);
l3 = get_line(p5, p6);
l4 = get_line(p7, p8);

% show the chosen lines in the image
figure;imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'y');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'y');
hold off;

% ToDo: compute the homography that affinely rectifies the image

vp1 = cross(l1, l2);
vp1 = [vp1(1) / vp1(3), vp1(2) / vp1(3), 1];

vp2 = cross(l3, l4);
vp2 = [vp2(1) / vp2(3), vp2(2) / vp2(3), 1];

coefficients = polyfit([vp1(1), vp2(1)], [vp1(2), vp2(2)], 1);
vl = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

Hp = [1 0 0; 0 1 0; vl];

I2 = apply_H(I, Hp);
% figure; imshow(uint8(I2));

% ToDo: compute the transformed lines lr1, lr2, lr3, lr4

lr1 = inv(Hp)' * l1';
lr2 = inv(Hp)' * l2';
lr3 = inv(Hp)' * l3';
lr4 = inv(Hp)' * l4';

% show the transformed lines in the transformed image
figure;imshow(uint8(I2));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');
hold off;

% ToDo: to evaluate the results, compute the angle between the different pair 
% of lines before and after the image transformation

disp('Angles after affine rectification:')

disp('Angle 1:');disp(get_angle(l1, l2));
disp('Angle 1 rectified:');disp(get_angle(lr1, lr2));

disp('Angle 2:');disp(get_angle(l3, l4));
disp('Angle 2 rectified:');disp(get_angle(lr3, lr4));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Metric Rectification

%% 3.1 Metric rectification after the affine rectification (stratified solution)

% ToDo: Metric rectification (after the affine rectification) using two non-parallel orthogonal line pairs
%       As evaluation method you can display the images (before and after
%       the metric rectification) with the chosen lines printed on it.
%       Compute also the angles between the pair of lines before and after
%       rectification.

lr1 = [lr1(1) / lr1(3), lr1(2)/lr1(3), 1];
lr2 = [lr2(1) / lr2(3), lr2(2)/lr2(3), 1];
lr3 = [lr3(1) / lr3(3), lr3(2)/lr3(3), 1];
lr4 = [lr4(1) / lr4(3), lr4(2)/lr4(3), 1];

lm1 = [lr1(1)*lr3(1), lr1(1)*lr3(2) + lr1(2)*lr3(1), lr1(2)*lr3(2)];
lm2 = [lr2(1)*lr4(1), lr2(1)*lr4(2) + lr2(2)*lr4(1), lr2(2)*lr4(2)];

A = [lm1(1:2); lm2(1:2)];
b = -[lm1(3); lm2(3)];
sol = linsolve(A, b);

S = [sol(1) sol(2); sol(2) 1];
R = chol(inv(S));

Ha = [R [0;0]; 0 0 1];

H3 = Ha*Hp;

I3 = apply_H(I, H3);

% Now we compute the lines from the transformed points
lr1 = inv(H3)' * l1';
lr2 = inv(H3)' * l2';
lr3 = inv(H3)' * l3';
lr4 = inv(H3)' * l4';

lr1 = [lr1(1) / lr1(3), lr1(2)/lr1(3), 1];
lr2 = [lr2(1) / lr2(3), lr2(2)/lr2(3), 1];
lr3 = [lr3(1) / lr3(3), lr3(2)/lr3(3), 1];
lr4 = [lr4(1) / lr4(3), lr4(2)/lr4(3), 1];

figure; imshow(uint8(I3));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');
hold off;

disp('Angles after metric rectification:')

disp('Angle parallel 1:');disp(get_angle(l1, l2));
disp('Angle parallel 1 rectified:');disp(get_angle(lr1, lr2));

disp('Angle paralle l2:');disp(get_angle(l3, l4));
disp('Angle parallel 2 rectified:');disp(get_angle(lr3, lr4));

disp('Angle ortho 1:');disp(get_angle(l1, l2));
disp('Angle ortho 1 rectified:');disp(get_angle(lr1, lr3));

disp('Angle ortho l2:');disp(get_angle(l2, l4));
disp('Angle ortho 2 rectified:');disp(get_angle(lr2, lr4));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. Affine and Metric Rectification of the left facade of image 0001
close all
% ToDo: Write the code that rectifies the left facade of image 0001 with
%       the stratified method (affine + metric). 
%       Crop the initial image so that only the left facade is visible.
%       Show the (properly) transformed lines that use in every step.


% choose the image points
I=imread('Data/0001_s.png');
A = load('Data/0001_s_info_lines.txt');

I = I(:, 1:480, :);

% indices of lines
[p1, p2] = get_points (614, A);
[p3, p4] = get_points (159, A);
[p5, p6] = get_points (645, A);
[p7, p8] = get_points (541, A);


% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points

l1 = get_line(p1, p2);
l2 = get_line(p3, p4);
l3 = get_line(p5, p6);
l4 = get_line(p7, p8);

% show the chosen lines in the image
figure;imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'y');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'y');
hold off;

% Affine rectification

vp1 = cross(l1, l2);
vp1 = [vp1(1) / vp1(3), vp1(2) / vp1(3), 1];

vp2 = cross(l3, l4);
vp2 = [vp2(1) / vp2(3), vp2(2) / vp2(3), 1];

coefficients = polyfit([vp1(1), vp2(1)], [vp1(2), vp2(2)], 1);
vl = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

Hp = [1 0 0; 0 1 0; vl];

I2 = apply_H(I, Hp);

lr1 = inv(Hp)' * l1';
lr2 = inv(Hp)' * l2';
lr3 = inv(Hp)' * l3';
lr4 = inv(Hp)' * l4';

lr1 = [lr1(1) / lr1(3), lr1(2)/lr1(3), 1];
lr2 = [lr2(1) / lr2(3), lr2(2)/lr2(3), 1];
lr3 = [lr3(1) / lr3(3), lr3(2)/lr3(3), 1];
lr4 = [lr4(1) / lr4(3), lr4(2)/lr4(3), 1];

% show the transformed lines in the transformed image
figure;imshow(uint8(I2));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');
hold off;

% Metric rectification

lr1 = [lr1(1) / lr1(3), lr1(2)/lr1(3), 1];
lr2 = [lr2(1) / lr2(3), lr2(2)/lr2(3), 1];
lr3 = [lr3(1) / lr3(3), lr3(2)/lr3(3), 1];
lr4 = [lr4(1) / lr4(3), lr4(2)/lr4(3), 1];

lm1 = [lr1(1)*lr3(1), lr1(1)*lr3(2) + lr1(2)*lr3(1), lr1(2)*lr3(2)];
lm2 = [lr2(1)*lr4(1), lr2(1)*lr4(2) + lr2(2)*lr4(1), lr2(2)*lr4(2)];

A = [lm1(1:2); lm2(1:2)];
b = -[lm1(3); lm2(3)];
sol = linsolve(A, b);

S = [sol(1) sol(2); sol(2) 1];
R = chol(inv(S));

Ha = [R [0;0]; 0 0 1];

H3 = Ha*Hp;

I3 = apply_H(I, H3);

% Now we compute the lines from the transformed points
lr1 = inv(H3)' * l1';
lr2 = inv(H3)' * l2';
lr3 = inv(H3)' * l3';
lr4 = inv(H3)' * l4';

lr1 = [lr1(1) / lr1(3), lr1(2)/lr1(3), 1];
lr2 = [lr2(1) / lr2(3), lr2(2)/lr2(3), 1];
lr3 = [lr3(1) / lr3(3), lr3(2)/lr3(3), 1];
lr4 = [lr4(1) / lr4(3), lr4(2)/lr4(3), 1];

figure; imshow(uint8(I3));
hold on;
t=1:0.1:10000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');
hold off;

disp('Angles after metric rectification:')

disp('Angle parallel 1:');disp(get_angle(l1, l2));
disp('Angle parallel 1 rectified:');disp(get_angle(lr1, lr2));

disp('Angle paralle l2:');disp(get_angle(l3, l4));
disp('Angle parallel 2 rectified:');disp(get_angle(lr3, lr4));

disp('Angle ortho 1:');disp(get_angle(l1, l2));
disp('Angle ortho 1 rectified:');disp(get_angle(lr1, lr3));

disp('Angle ortho 2:');disp(get_angle(l2, l4));
disp('Angle ortho 2 rectified:');disp(get_angle(lr2, lr4));



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5. OPTIONAL: Metric Rectification in a single step
% % Use 5 pairs of orthogonal lines (pages 55-57, Hartley-Zisserman book)
% % indices of lines
% close all
% % choose the image points
% I=imread('Data/0000_s.png');
% A = load('Data/0000_s_info_lines.txt');
% 
% % indices of lines
% [p1, p2] = get_points (424, A);
% [p3, p4] = get_points (240, A);
% [p5, p6] = get_points (712, A);
% [p7, p8] = get_points (565, A);
% [p9, p10] = get_points (227, A);
% [p11, p12] = get_points (534, A);
% 
% 
% % ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points
% 
% l1 = get_line(p1, p2);
% l2 = get_line(p3, p4);
% l3 = get_line(p5, p6);
% l4 = get_line(p7, p8);
% l5 = get_line(p9, p10);
% l6 = get_line(p11, p12);
% 
% 
% lm1 = [l1(1)*l3(1),(l1(1)*l3(2)+l1(2)*l3(1))/2, l1(2)*l3(2), (l1(1)*l3(3)+l1(3)*l3(1))/2, (l1(2)*l3(3)+l1(3)*l3(2))/2, l1(3)*l3(3)];
% lm2 = [l2(1)*l4(1),(l2(1)*l4(2)+l2(2)*l4(1))/2, l2(2)*l4(2), (l2(1)*l4(3)+l2(3)*l4(1))/2, (l2(2)*l4(3)+l2(3)*l4(2))/2, l2(3)*l4(3)];
% lm3 = [l1(1)*l4(1),(l1(1)*l4(2)+l1(2)*l4(1))/2, l1(2)*l4(2), (l1(1)*l4(3)+l1(3)*l4(1))/2, (l1(2)*l4(3)+l1(3)*l4(2))/2, l1(3)*l4(3)];
% lm4 = [l2(1)*l3(1),(l2(1)*l3(2)+l2(2)*l3(1))/2, l2(2)*l3(2), (l2(1)*l3(3)+l2(3)*l3(1))/2, (l2(2)*l3(3)+l2(3)*l3(2))/2, l2(3)*l3(3)];
% lm5 = [l5(1)*l6(1),(l5(1)*l6(2)+l5(2)*l6(1))/2, l5(2)*l6(2), (l5(1)*l6(3)+l5(3)*l6(1))/2, (l5(2)*l6(3)+l5(3)*l6(2))/2, l5(3)*l6(3)];
% 
% A = [lm1; lm2; lm3; lm4; lm5];
% 
% [U, D, V] = svd(A);
% 
% C = V(:, 6);
% C = [C(1), C(2)/2, C(4)/2; C(2)/2, C(3), C(5)/2; C(4)/2, C(5)/2, C(6)];
% 
% K = chol(inv(C(1:2, 1:2)));
% v = inv(C(1:2, 1:2)) * C(1:2, 3);
% 
% H = [K, [0; 0]; v', 1];
% 
% I5 = apply_H(I, H);
% figure; imshow(uint8(I5));
