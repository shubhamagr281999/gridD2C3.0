%% reading images
img1=imread('arena.png');
img1=im2double(img1);

%% taking 12 physical correspondence point
for i=1:2
    figure(1);
    imshow(img1);
    [x1(i),y1(i)]=ginput(1);
end



