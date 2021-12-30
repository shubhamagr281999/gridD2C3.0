%global varibales for pose and speed
global n_agents;
global current_pose;
global img;
global bot_marker;

n_agents=8;
current_pose=zeros(n_agents,3);
bot_marker=uint8(zeros(n_agents,44,44));
%creating maze with IP
img=uint8(zeros(54*14,54*15)+255);
%making the maze image
mask_border=uint8(zeros(54,54)+255);
mask_border(:,[1,2,53,54])=uint8(0);
mask_border([1,2,53,54],:)=uint8(0);
for i=1:14
    img(1:54,(i*54+1):(i+1)*54)=mask_border;
end
img(55:54*2,:)=img(1:54,:);
img((54*4+1):54*6,:)=img(1:54*2,:);
for i=1:14
    if((i-4*floor(i/4))==1 || (i-4*floor(i/4))==2)
        img(54*2+1:54*3,(i*54+1):(i+1)*54)=mask_border;
    end
end
img(54*3+1:54*4,:)=img(54*2+1:54*3,:);
img(54*6+1:54*8,:)=img(54*2+1:54*4,:);
img(54*8+1:54*14,:)=img(1:54*6,:);
img(54*4+1:54*5,1:54)=mask_border;
img(54*9+1:54*10,1:54)=mask_border;
imwrite(img,'maze.png')
%with the above operations maze image is ready with each grid of size 54*54
%pixels

bot_marker(1,:,:)=rgb2gray(imread('bot0.png'));
bot_marker(2,:,:)=rgb2gray(imread('bot1.png'));
bot_marker(3,:,:)=rgb2gray(imread('bot2.png'));
bot_marker(4,:,:)=rgb2gray(imread('bot3.png'));
bot_marker(5,:,:)=rgb2gray(imread('bot4.png'));
bot_marker(6,:,:)=rgb2gray(imread('bot5.png'));
bot_marker(7,:,:)=rgb2gray(imread('bot6.png'));
bot_marker(8,:,:)=rgb2gray(imread('bot7.png'));

figure (1)
hold on
imshow(img)
%% creating subscriber here
rosshutdown
rosinit('http://bot:11311/')
sub1 = rossubscriber('/poses',@pose_callback);
pause(1)

