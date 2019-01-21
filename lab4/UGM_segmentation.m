close all;
clc;

im_name='Data/0001_S.png';

% TODO: Update library path
% Add  library paths
addpath('UGM');

%Set model parameters
%cluster color
K=4; % Number of color clusters (=number of states of hidden variables)

%Pair-wise parameters
smooth_term=[0.0 1]; % Potts Model

%Load images
im = imread(im_name);

NumFils = size(im,1);
NumCols = size(im,2);

%Convert to LAB colors space
% TODO: Uncomment if you want to work in the LAB space
%
% im = RGB2Lab(im);

%Preparing data for GMM fiting
im=double(im);
x=reshape(im,[size(im,1)*size(im,2) size(im,3)]);
gmm_color = gmdistribution.fit(x,K);
mu_color=gmm_color.mu;
% TODO: define the unary energy term: data_term
data_term = gmm_color.posterior(x);
% [~,c] = max(data_term,[],2);

% nodePot = P( color at pixel 'x' | Cluster color 'c' )  
nodePot = data_term;

%Building 4-grid
%Build UGM Model for 4-connected segmentation
disp('create UGM model');

% Create UGM data
[edgePot,edgeStruct] = CreateGridUGMModel(NumFils, NumCols, K ,smooth_term);

if ~isempty(edgePot)
    % color clustering
    disp('Clustering'); tic;
    [~,c] = max(reshape(data_term,[NumFils*NumCols K]),[],2); toc;
    im_c= reshape(mu_color(c,:),size(im));
    
    % Call different UGM inference algorithms
    disp('Loopy Belief Propagation'); tic;
    [nodeBelLBP,edgeBelLBP,logZLBP] = UGM_Infer_LBP(nodePot,edgePot,edgeStruct);toc;
    [~, i] = max(nodeBelLBP,[],2);
    im_lbp= reshape(mu_color(i,:), size(im));
    
    % Max-sum
    disp('Max-sum'); tic;
    decodeLBP = UGM_Decode_LBP(nodePot,edgePot,edgeStruct);
    im_bp= reshape(mu_color(decodeLBP,:), size(im));
    toc;
   
    % ICM
    disp('ICM'); tic;
    decodeICM = UGM_Decode_ICM(nodePot,edgePot,edgeStruct);
    im_icm= reshape(mu_color(decodeICM,:), size(im));
    toc;
    
    % Sample
    burnIn = 1000;
    disp('Sample'); tic;
    decodeSample = UGM_Decode_Sample(nodePot, edgePot, edgeStruct,@UGM_Sample_Gibbs,burnIn);
    im_Sample= reshape(mu_color(decodeSample,:), size(im));
    toc;
    
    % Max Of Marginals
    disp('MaxOfMarginals'); tic;
    decodeMaxOfMarginals = UGM_Decode_MaxOfMarginals(nodePot,edgePot,edgeStruct,@UGM_Infer_LBP);
    im_MaxOfMarginals= reshape(mu_color(decodeMaxOfMarginals,:), size(im));
    toc;
    
    % Graph Cut
    if K == 2
        disp('Graph Cut'); tic;
        decodeGraphCut = UGM_Decode_GraphCut(nodePot,edgePot,edgeStruct);
        im_GraphCut= reshape(mu_color(decodeGraphCut,:), size(im));
        toc;
    end
    
    figure
%     subplot(2,2,1),imshow(Lab2RGB(im));xlabel('Original');
%     subplot(2,2,2),imshow(Lab2RGB(im_c),[]);xlabel('Clustering without GM');
%     subplot(2,2,3),imshow(Lab2RGB(im_bp),[]);xlabel('Max-Sum');
%     subplot(2,2,4),imshow(Lab2RGB(im_lbp),[]);xlabel('Loopy Belief Propagation');
    subplot(3,3,8),imshow(im/255);xlabel('Original');
    subplot(3,3,1),imshow(im_c/255);xlabel('Clustering without GM');
    subplot(3,3,2),imshow(im_bp/255);xlabel('Max-Sum');
    subplot(3,3,3),imshow(im_lbp/255);xlabel('Loopy Belief Propagation');
    subplot(3,3,4),imshow(im_icm/255);xlabel('ICM');
    subplot(3,3,5),imshow(im_Sample/255);xlabel('Gibbs Sample');
    subplot(3,3,6),imshow(im_MaxOfMarginals/255);xlabel('MaxOfMarginals');
    if K == 2
        subplot(3,3,7),imshow(im_GraphCut/255);xlabel('Graph Cut');
    end
else
    error('You have to implement the CreateGridUGMModel.m function');
end