function cmd_vel_callback(~,msg)
tic
 global n_agents;
 global bot_cmd_vel;
 global img;
 global aruco;
 global current_pose;
 global pub;
 img1=img;
 for i=1:n_agents
 	bot_cmd_vel(i,1)=msg.Poses(i).Position.X;
 	bot_cmd_vel(i,2)=msg.Poses(i).Position.Y;
 	bot_cmd_vel(i,3)=msg.Poses(i).Position.Z;
 end
 %image update logic
 for i=1:1
     x=floor(current_pose(i,1)+bot_cmd_vel(i,1)*0.1);
     y=floor(current_pose(i,2)+bot_cmd_vel(i,2)*0.1);
     yaw=floor(current_pose(i,3)+bot_cmd_vel(i,3)*0.1);
     aruco_rot=imrotate(reshape(aruco(i,:,:),[44,44]),yaw);
     x=(x-ceil(size(aruco_rot,1)/2) + 1):(x+floor(size(aruco_rot,1)/2));
     y=(y-ceil(size(aruco_rot,1)/2) + 1):(y+floor(size(aruco_rot,1)/2));
     img1(x,y)=aruco_rot;
 end
msg = rosmessage('sensor_msgs/Image');
msg.Encoding = 'mono8';
img1=padarray(reshape(aruco(1,:,:),[44,44]),[30,60]);
writeImage(msg,img1);
send(pub,msg)
toc
end
