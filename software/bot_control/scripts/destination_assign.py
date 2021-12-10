#!/usr/bin/env python
import pandas as pd
import rospy
from bot_control.msg import dest_id, pkg_flag

class destination_assign:
    def __init__(self):
        self.dicti = {0: 'Mumbai', 1: 'Delhi', 2: 'Kolkata', 3: 'Chennai', 4: 'Bengaluru', 5: 'Hyderabad', 6: 'Pune', 7: 'Ahmedabad', 8: 'Jaipur'}
        self.key_list = list(self.dicti.keys())
        self.val_list = list(self.dicti.values())
        self.excel = pd.read_excel(r'~/catkin_ws/src/gridD2C3.0/Sample Data.xls')
        self.destination= self.excel['Destination'].tolist()

        self.LS= self.excel['Induct Station'].tolist()
        self.list_split()
        self.pub_destination=rospy.Publisher('/pkg_dest_id',dest_id,queue_size=1)
        self.sub_station=rospy.Subscriber("/pkg_received", pkg_flag, self.assign)
        self.dest_id_msg = dest_id()
        # print(self.destination)

    def list_split(self):
        self.induct1 = []
        self.induct2 = []
        for i in range(len(self.LS)):
            if self.LS[i] == 1:
                self.induct1.append(self.destination[i])
            elif self.LS[i] ==2:
                self.induct2.append(self.destination[i])
        # print(self.induct1)
    def assign(self,msg):
        if msg.LS == 1:
            self.dest_id_msg.LS = 1
            self.dest_id_msg.bot_num = msg.bot_num
            self.dest_id_msg.dest_id = self.key_list[self.val_list.index(self.induct1[0])]
            self.induct1.pop(0)
            self.pub_destination.publish(self.dest_id_msg)
            # print(self.dest_id_msg.dest_id)
        elif msg.LS == 2:
            self.dest_id_msg.LS = 2
            self.dest_id_msg.bot_num = msg.bot_num
            self.dest_id_msg.dest_id = self.key_list[self.val_list.index(self.induct2[0])]
            self.induct2.pop(0)
            self.pub_destination.publish(self.dest_id_msg)


if __name__ == '__main__':
    rospy.init_node('destination_assign')
    rospy.loginfo('Assigner node created')
    assigner = destination_assign()
    rospy.spin()