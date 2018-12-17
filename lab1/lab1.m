close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 1: Image rectification


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Applying image transformations

% ToDo: create the function  "apply_H" that gets as input a homography and
% an image and returns the image transformed by the homography.
% The size of the transformed image has to be automatically set so as to 
% contain the whole transformed image.
% At some point you will need to interpolate the image values at some points,
% you may use the Matlab function "interp2" for that.


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
    disp("The matrix are equal")
else 
    disp("They are not the same")
end

       
% ToDo: verify that the proper sequence of the four previous
% transformations over the image I produces the same image I2 as before
if ( isequal(I2,I3))
    disp("The images are equal")
else
    disp("The images are not equal")
end


%% 1.3 Projective transformations (homographies)

% ToDo: generate a matrix H which produces a projective transformation
close all
H2 = H1;
H2(3,1:2) = [0,0.0005];
I2 = apply_H(I, H2);
figure; imshow(I); figure; imshow(uint8(I2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Affine Rectification

% choose the image points
I=imread('Data/0000_s.png');
A = load('Data/0000_s_info_lines.txt');

% indices of lines
i = 424;
p1 = [A(i,1) A(i,2) 1]';
p2 = [A(i,3) A(i,4) 1]';
i = 240;
p3 = [A(i,1) A(i,2) 1]';
p4 = [A(i,3) A(i,4) 1]';
i = 712;
p5 = [A(i,1) A(i,2) 1]';
p6 = [A(i,3) A(i,4) 1]';
i = 565;
p7 = [A(i,1) A(i,2) 1]';
p8 = [A(i,3) A(i,4) 1]';

% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points

coefficients = polyfit([p1(1), p2(1)], [p1(2), p2(2)], 1);
l1 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p3(1), p4(1)], [p3(2), p4(2)], 1);
l2 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p5(1), p6(1)], [p5(2), p6(2)], 1);
l3 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p7(1), p8(1)], [p7(2), p8(2)], 1);
l4 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

% show the chosen lines in the image
figure;imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'y');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'y');

% ToDo: compute the homography that affinely rectifies the image

vp1 = cross(l1, l2);
vp1 = [vp1(1) / vp1(3), vp1(2) / vp1(3), 1];

vp2 = cross(l3, l4);
vp2 = [vp2(1) / vp2(3), vp2(2) / vp2(3), 1];

coefficients = polyfit([vp1(1), vp2(1)], [vp1(2), vp2(2)], 1);
vl = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

Hp = [1 0 0; 0 1 0; vl];

I2 = apply_H(I, Hp);
figure; imshow(uint8(I2));

% ToDo: compute the transformed lines lr1, lr2, lr3, lr4

% First we transform the points
pr1 = Hp*p1;
pr1 = [pr1(1)/ pr1(3), pr1(2)/pr1(3), 1];
pr2 = Hp*p2;
pr2 = [pr2(1)/ pr2(3), pr2(2)/pr2(3), 1];
pr3 = Hp*p3;
pr3 = [pr3(1)/ pr3(3), pr3(2)/pr3(3), 1];
pr4 = Hp*p4;
pr4 = [pr4(1)/ pr4(3), pr4(2)/pr4(3), 1];
pr5 = Hp*p5;
pr5 = [pr5(1)/ pr5(3), pr5(2)/pr5(3), 1];
pr6 = Hp*p6;
pr6 = [pr6(1)/ pr6(3), pr6(2)/pr6(3), 1];
pr7 = Hp*p7;
pr7 = [pr7(1)/ pr7(3), pr7(2)/pr7(3), 1];
pr8 = Hp*p8;
pr8 = [pr8(1)/ pr8(3), pr8(2)/pr8(3), 1];

% Now we compute the lines from the transformed points
coefficients = polyfit([pr1(1), pr2(1)], [pr1(2), pr2(2)], 1);
lr1 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr3(1), pr4(1)], [pr3(2), pr4(2)], 1);
lr2 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr5(1), pr6(1)], [pr5(2), pr6(2)], 1);
lr3 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr7(1), pr8(1)], [pr7(2), pr8(2)], 1);
lr4 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

% show the transformed lines in the transformed image
figure;imshow(uint8(I2));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');

% ToDo: to evaluate the results, compute the angle between the different pair 
% of lines before and after the image transformation
cos1 = dot(l1(1:2), l2(1:2)) / (norm(l1(1:2)) * norm(l2(1:2)));
cosr1 = dot(lr1(1:2), lr2(1:2)) / (norm(lr1(1:2)) * norm(lr2(1:2)));
angle1 = acos(cos1);
disp("Angle 1:");disp(angle1);
angler1 = acos(cosr1);
disp("Angle 1 rectified:");disp(angler1);

cos2 = dot(l3(1:2), l4(1:2)) / (norm(l3(1:2)) * norm(l4(1:2)));
angle2 = acos(cos2);
disp("Angle 2:");disp(angle2);
cosr2 = dot(lr3(1:2), lr4(1:2)) / (norm(lr3(1:2)) * norm(lr4(1:2)));
angler2 = acos(cosr2);
disp("Angle 2 rectified:");disp(angler2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Metric Rectification

%% 3.1 Metric rectification after the affine rectification (stratified solution)

% ToDo: Metric rectification (after the affine rectification) using two non-parallel orthogonal line pairs
%       As evaluation method you can display the images (before and after
%       the metric rectification) with the chosen lines printed on it.
%       Compute also the angles between the pair of lines before and after
%       rectification.

lm1 = [lr1(1)*lr3(1), lr1(1)*lr3(2) + lr1(2)*lr3(1), lr1(2)*lr3(2)];
lm2 = [lr2(1)*lr4(1), lr2(1)*lr4(2) + lr2(2)*lr4(1), lr2(2)*lr4(2)];

A = [lm1(1:2); lm2(1:2)];
b = [lm1(3); lm2(3)];
sol = linsolve(A, b);

sv = [sol' 1];

S = [sv(1) sv(2); sv(2) sv(3)];
R = chol(S);

Ha = inv([R [0;0]; 0 0 1]);

close all
I3 = apply_H(I, Ha*Hp');
figure; imshow(uint8(I3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. Affine and Metric Rectification of the left facade of image 0001

% ToDo: Write the code that rectifies the left facade of image 0001 with
%       the stratified method (affine + metric). 
%       Crop the initial image so that only the left facade is visible.
%       Show the (properly) transformed lines that use in every step.

close all
clear all
clc

% choose the image points
I=imread('Data/0001_s.png');
A = load('Data/0001_s_info_lines.txt');

% indices of lines
i = 614;
p1 = [A(i,1) A(i,2) 1]';
p2 = [A(i,3) A(i,4) 1]';
i = 159;
p3 = [A(i,1) A(i,2) 1]';
p4 = [A(i,3) A(i,4) 1]';
i = 645;
p5 = [A(i,1) A(i,2) 1]';
p6 = [A(i,3) A(i,4) 1]';
i = 541;
p7 = [A(i,1) A(i,2) 1]';
p8 = [A(i,3) A(i,4) 1]';

% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points

coefficients = polyfit([p1(1), p2(1)], [p1(2), p2(2)], 1);
l1 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p3(1), p4(1)], [p3(2), p4(2)], 1);
l2 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p5(1), p6(1)], [p5(2), p6(2)], 1);
l3 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([p7(1), p8(1)], [p7(2), p8(2)], 1);
l4 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

% show the chosen lines in the image
figure;imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'y');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'y');

% ToDo: compute the homography that affinely rectifies the image

vp1 = cross(l1, l2);
vp1 = [vp1(1) / vp1(3), vp1(2) / vp1(3), 1];

vp2 = cross(l3, l4);
vp2 = [vp2(1) / vp2(3), vp2(2) / vp2(3), 1];

coefficients = polyfit([vp1(1), vp2(1)], [vp1(2), vp2(2)], 1);
vl = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

H = [1 0 0; 0 1 0; vl];

I2 = apply_H(I, H);
figure; imshow(uint8(I2));

% ToDo: compute the transformed lines lr1, lr2, lr3, lr4

pr1 = H*p1;
pr1 = [pr1(1)/ pr1(3), pr1(2)/pr1(3), 1];
pr2 = H*p2;
pr2 = [pr2(1)/ pr2(3), pr2(2)/pr2(3), 1];
pr3 = H*p3;
pr3 = [pr3(1)/ pr3(3), pr3(2)/pr3(3), 1];
pr4 = H*p4;
pr4 = [pr4(1)/ pr4(3), pr4(2)/pr4(3), 1];
pr5 = H*p5;
pr5 = [pr5(1)/ pr5(3), pr5(2)/pr5(3), 1];
pr6 = H*p6;
pr6 = [pr6(1)/ pr6(3), pr6(2)/pr6(3), 1];
pr7 = H*p7;
pr7 = [pr7(1)/ pr7(3), pr7(2)/pr7(3), 1];
pr8 = H*p8;
pr8 = [pr8(1)/ pr8(3), pr8(2)/pr8(3), 1];

coefficients = polyfit([pr1(1), pr2(1)], [pr1(2), pr2(2)], 1);
lr1 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr3(1), pr4(1)], [pr3(2), pr4(2)], 1);
lr2 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr5(1), pr6(1)], [pr5(2), pr6(2)], 1);
lr3 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

coefficients = polyfit([pr7(1), pr8(1)], [pr7(2), pr8(2)], 1);
lr4 = [coefficients(1)/coefficients(2) -1/coefficients(2) 1];

% show the transformed lines in the transformed image
figure;imshow(uint8(I2));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');

% ToDo: to evaluate the results, compute the angle between the different pair 
% of lines before and after the image transformation
cos1 = dot(l1(1:2), l2(1:2)) / (norm(l1(1:2)) * norm(l2(1:2)));
cosr1 = dot(lr1(1:2), lr2(1:2)) / (norm(lr1(1:2)) * norm(lr2(1:2)));
angle1 = acos(cos1);
disp("Angle 1:");disp(angle1);
angler1 = acos(cosr1);
disp("Angle 1 rectified:");disp(angler1);

cos2 = dot(l3(1:2), l4(1:2)) / (norm(l3(1:2)) * norm(l4(1:2)));
angle2 = acos(cos2);
disp("Angle 2:");disp(angle2);
cosr2 = dot(lr3(1:2), lr4(1:2)) / (norm(lr3(1:2)) * norm(lr4(1:2)));
angler2 = acos(cosr2);
disp("Angle 2 rectified:");disp(angler2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5. OPTIONAL: Metric Rectification in a single step
% Use 5 pairs of orthogonal lines (pages 55-57, Hartley-Zisserman book)


lm1 = [lr1(1)*lr3(1), lr1(1)*lr3(2) + lr1(2)*lr3(1), lr1(2)*lr3(2)];
lm2 = [lr2(1)*lr4(1), lr2(1)*lr4(2) + lr2(2)*lr4(1), lr2(2)*lr4(2)];

A = [lm1(1:2); lm2(1:2)];
b = [lm1(3); lm2(3)];
sol = linsolve(A, b);

sv = [sol' 1];

S = [sv(1) sv(2); sv(2) sv(3)];
R = chol(S);

Ha = inv([R [0;0]; 0 0 1]);

close all
I3 = apply_H(I, Ha*Hp');
figure; imshow(uint8(I3));

