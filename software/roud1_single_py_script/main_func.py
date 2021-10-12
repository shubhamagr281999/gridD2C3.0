import cv2
from math import sqrt, atan, pi
import numpy as np
import flask
import time


class grid:
    def __init__(self):
        self.current_pose=np.array([[276,26,0],[310,26,0.0],[338,29,0.0],[369,29,0.0]])
        self.goal_pose=[[276,26,pi/2],[310,26,0.0],[338,29,0.0],[369,29,0.0]]
        self.lastError = np.zeros((4,1))
        self.lastError_small=np.zeros((4,1))
        self.sumError = np.zeros((4,1))

        self.l=0.095
        self.r=0.034
        self.s=1

        self.msg=["0,0,1", "0,0,1", "0,0,1", "0,0,1"]

        self.count=0
        self.mission_bot=0
        self.vid = cv2.VideoCapture(0)
        self.flip_desired_yaw=[pi/2,pi/2,-pi/2,-pi/2]
        # params to tune
        self.time_for_flipping=2
        self.lin_threshold_SE=200
        self.yaw_threshold_SE=2

        self.kp_lin = 0.18
        self.ki_lin = 0.0
        self.kd_lin = 0.0

        self.kp_angle = -75
        self.ki_angle = 0.0
        self.kd_angle = -25.0

        self.kp_angle_soft = -40
        self.ki_angle_soft = 0.0
        self.kd_angle_soft = -20.0

        self.max_vel_lin=6
        self.max_vel_ang=50
        self.lin_threshold=5
        self.yaw_threshold=0.1
        self.intergral_windup_yaw=20
        self.intergral_windup_lin=15

    def resetValues(self,bot):
        self.lastError[bot][0] = 0
        self.lastError_small[bot][0]=0
        self.sumError[bot][0]= 0

    def change_pose(self,bot,y,x,yaw):
        # if(abs(y-self.current_pose[bot][1])<self.lin_threshold_SE):
        if(y!=0):
            self.current_pose[bot][1]=0.93*y
        # if(abs(x-self.current_pose[bot][0])<self.lin_threshold_SE):
        if(x!=0):
            self.current_pose[bot][0]=x
        if(yaw!=0):
            self.current_pose[bot][2]=yaw

    def correct_diff_yaw(self, diff_yaw):
        if diff_yaw < -1*pi:
            return self.correct_diff_yaw(diff_yaw+2*pi)
        if diff_yaw > pi:
            return self.correct_diff_yaw(diff_yaw-2*pi)
        return diff_yaw

    def angle(self,y,x):
        x=x+0.000001
        if(x>0 and y>0):
            return atan(y/x)
        elif(x<0 and y>0):
            return (pi +atan(y/x))
        elif(x<0 and y<0):
            return (atan(y/x)-pi)
        else:
            return atan(y/(x+0.00001))

    def estimation(self):
        ret, img = self.vid.read()
        ret, img1 = self.vid.read()
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        #now starting to localise bot wrt to the ids
        img=img[55:375,25:630]
        img1=img1[55:375,25:630]
        img=cv2.resize(img,(1815,1020))

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
        a=np.where(ids==10)
        if a[0].size==1:
            # print('1')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(100,65,65),-1)            
            self.change_pose(0,cY/3,cX/3,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[0][0]),int(self.current_pose[0][1])),10,(100,64,65),-1)

        a=np.where(ids==20)
        if a[0].size==1:
            print('2')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(100,64,100),-1)
           
            self.change_pose(1,cY/3,cX/3,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[1][0]),int(self.current_pose[1][1])),10,(100,64,100),-1)
            # img1=cv2.circle(img1,self.current_pose[1][0],self.current_pose[1][1],10,(100,64,100),-1)

        a=np.where(ids==30)
        if a[0].size==1:
            print('3')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,60,100),-1)  
            
            self.change_pose(2,cY/3,cX/3,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[2][0]),int(self.current_pose[2][1])),10,(100,64,120),-1)
            # img1=cv2.circle(img1,self.current_pose[2][0],self.current_pose[2][1],10,(50,60,100),-1)

        a=np.where(ids==40)
        if a[0].size==1:
            print('4')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,100,80),-1)
            img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(50,100,80),-1)
            self.change_pose(3,cY/3,cX/3,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[3][0]),int(self.current_pose[3][1])),10,(100,64,165),-1)
        
        # cv2.imshow('original_image', img)
        # cv2.waitKey(1)
        cv2.imshow('processed_image',img1)
        cv2.waitKey(1)
        # cv2.imwrite('arena.png',img1)

    def flipmotor(self,bot):
        print('fliiping pracel for bot', (bot+1) )
        diff_yaw=self.flip_desired_yaw[bot]-self.current_pose[bot][2]
        self.resetValues(bot)
        print(diff_yaw, self.current_pose[bot][2])
        while(abs(diff_yaw)>self.yaw_threshold):
            self.estimation()
            diff_yaw=self.correct_diff_yaw(self.flip_desired_yaw[bot]-self.current_pose[bot][2])
            control_signal_angle=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError[bot][0])+self.ki_angle*(self.sumError[bot][0])
            self.lastError[bot][0]=diff_yaw
            if(abs(self.sumError[bot][0]+diff_yaw)<self.intergral_windup_yaw):
                self.sumError[bot][0]=self.sumError[bot][0]+diff_yaw            
            v=0
            if(control_signal_angle<self.max_vel_ang):
                w=control_signal_angle
            else:
                w=self.max_vel_ang
            wl=(2*v-self.l*w)/(2*self.r) 
            wr=(2*v+self.l*w)/(2*self.r) 
            self.msg[bot]="{},{},{}".format(wl,wr,self.s)
            if(abs(wl)<15 or abs(wr)<15):
                self.msg[bot]="0,0,1"
                break
            print(self.msg[bot],'yaw:',self.current_pose[bot][2],'diff_yaw:',diff_yaw,v,'omega:',w) 
        self.msg[bot]="0,0,2"
        time.sleep(self.time_for_flipping)
        self.msg[bot]="0,0,1"

    def goal(self):
        if self.count==0:
            self.goal_pose[0][0]=272
            self.goal_pose[0][1]=269
        elif self.count==1:
            self.goal_pose[0][0]=48
            self.goal_pose[0][1]=260
        elif self.count==2:
            self.flipmotor(0)
            self.goal_pose[0][0]=272
            self.goal_pose[0][1]=269
        elif self.count==3:
            self.goal_pose[0][0]=276
            self.goal_pose[0][1]=26

        elif self.count==4:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[1][0]=298
            self.goal_pose[1][1]=292
        elif self.count==5:
            self.goal_pose[1][0]=48
            self.goal_pose[1][1]=293
        elif self.count==6:
            self.flipmotor(1)
            self.goal_pose[1][0]=298
            self.goal_pose[1][1]=292
        elif self.count==7:
            self.goal_pose[1][0]=310
            self.goal_pose[1][1]=26

        elif self.count==8:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[2][0]=329
            self.goal_pose[2][1]=292
        elif self.count==9:
            self.goal_pose[2][0]=547
            self.goal_pose[2][1]=303
        elif self.count==10:
            self.flipmotor(2)
            self.goal_pose[2][0]=329
            self.goal_pose[2][1]=292
        elif self.count==11:
            self.goal_pose[2][0]=338
            self.goal_pose[2][1]=29

        elif self.count==12:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[3][0]=355
            self.goal_pose[3][1]=269
        elif self.count==13:
            self.goal_pose[3][0]=549
            self.goal_pose[3][1]=272
        elif self.count==14:
            self.flipmotor(3)
            self.goal_pose[3][0]=355
            self.goal_pose[3][1]=269
        elif self.count==15:
            self.goal_pose[3][0]=369
            self.goal_pose[3][1]=29
        if(self.count<16):
            print('############################### new goal sent. Count:',self.count)
            self.PID(self.mission_bot)


    def PID(self,bot):
        self.estimation()
        path_angle=self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
        diff_distance=sqrt((self.current_pose[bot][0]-self.goal_pose[bot][0])**2+(self.current_pose[bot][1]-self.goal_pose[bot][1])**2)
        diff_yaw=self.correct_diff_yaw(path_angle-self.current_pose[bot][2])
        self.resetValues(bot)        
        while (abs(diff_yaw)>self.yaw_threshold):
            self.estimation()
            path_angle = self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
            diff_yaw=self.correct_diff_yaw(path_angle-self.current_pose[bot][2])
            control_signal_angle=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError[bot][0])+self.ki_angle*(self.sumError[bot][0])
            self.lastError[bot][0]=diff_yaw
            if(abs(self.sumError[bot][0]+diff_yaw)<self.intergral_windup_yaw):
                self.sumError[bot][0]=self.sumError[bot][0]+diff_yaw            
            v=0
            if(control_signal_angle<self.max_vel_ang):
                w=control_signal_angle
            else:
                w=self.max_vel_ang
            wl=(2*v-self.l*w)/(2*self.r) 
            wr=(2*v+self.l*w)/(2*self.r) 
            print(v,w)
            # print(wl,wr)
            self.msg[bot]="{},{},{}".format(1.02*wl,wr,self.s)
            if(abs(wl)<15 or abs(wr)<15):
                self.msg[bot]="0,0,1"
                break
            print(self.msg[bot],'path_angle:',path_angle,'yaw:',self.current_pose[bot][2],'diff_yaw:',diff_yaw,v,'omega:',w)

        #moving towards goal        
        self.resetValues(bot)
        path_angle=self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
        diff_distance=sqrt((self.current_pose[bot][0]-self.goal_pose[bot][0])**2+(self.current_pose[bot][1]-self.goal_pose[bot][1])**2)
        while(diff_distance>self.lin_threshold):
            self.estimation()
            path_angle= self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
            diff_yaw=self.correct_diff_yaw(path_angle-self.current_pose[bot][2])
            diff_distance=sqrt((self.current_pose[bot][0]-self.goal_pose[bot][0])**2+(self.current_pose[bot][1]-self.goal_pose[bot][1])**2)
            control_signal_dist=self.kp_lin*(diff_distance)+self.kd_lin*(diff_distance-self.lastError[bot][0])+self.ki_lin*self.sumError[bot][0]
            control_signal_angle=self.kp_angle_soft*(diff_yaw)+self.kd_angle_soft*(diff_yaw-self.lastError_small[bot][0])
            self.lastError[bot][0]=diff_distance
            self.lastError_small[bot][0]=diff_yaw
            if(abs(self.sumError[bot][0]+diff_distance)<self.intergral_windup_lin):
                self.sumError[bot][0]=self.sumError[bot][0]+diff_distance
            if(control_signal_dist<self.max_vel_lin):
                v=control_signal_dist
            else:
                v=self.max_vel_lin
            if(control_signal_angle<self.max_vel_ang):
                w=control_signal_angle
            else:
                w=self.max_vel_ang
            wl=(2*v-self.l*w)/(2*self.r) 
            wr=(2*v+self.l*w)/(2*self.r) 
            print(v,w)
            # print(wl,wr)
            self.msg[bot]="{},{},{}".format(1.02*wl,wr,self.s)
            if(abs(wl)<15 or abs(wr)<15):
                self.msg[bot]="0,0,1"
                break
            print(self.msg[bot],'path_angle:',path_angle,'yaw:',self.current_pose[bot][2],'diff_yaw:',diff_yaw,v,'omega:',w)

        self.resetValues(bot)
        self.msg[bot]="0,0,1"
        self.count=self.count+1
        self.goal()
    

if __name__ == '__main__':
    grid_obj=grid()
    
    app = flask.Flask(__name__)
    app.config["DEBUG"] = True

    @app.route('/1', methods=['GET'])
    def home():
        return grid_obj.msg[0]

    @app.route('/2', methods=['GET'])
    def home1():
        return grid_obj.msg[1]

    @app.route('/3', methods=['GET'])
    def home2():
        return grid_obj.msg[2]

    @app.route('/4', methods=['GET'])
    def home3():
        return grid_obj.msg[3]

    app.run(host="192.168.89.25")
    grid_obj.goal()
    #to keep the function running in loop