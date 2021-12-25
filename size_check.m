%% reading images
img1=rgb2gray(imread('maze.png'));
% img1=im2double(img1);

[x,y]=size(img1);
% img1_NN=zeros(y,x);
% img1_BI=zeros(y,x);

%% trimmed maze
img=img1(2:757,279:1088);
figure
imshow(img)

