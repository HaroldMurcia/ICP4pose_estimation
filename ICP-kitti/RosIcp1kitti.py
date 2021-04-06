#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import copy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
#import tila1 as ptstoxyz
import os
import time
from nav_msgs.msg import Odometry
msg = Odometry()
import matplotlib.pyplot as plt
np.set_printoptions(precision=4)


#from utils.ScanContextManager import *
#from utils.PoseGraphManager import *
#from utils.UtilsMisc import *
import utils.UtilsPointcloud as Ptutils
import utils.ICP as ICP
import utils.pointcloud as ptstoxyz

class laser_subs(object): #Para crear clase
 
    def __init__(self, name):
     
        self.num_icp_points = 5000 #number of points for each cloud
        self.num_rings = 20
        self.num_sectors = 60 
        self.num_candidates = 10
        self.try_gap_loop_detection = 10
        self.loop_threshold = 0.11    #threshold
        self.save_gap = 300
       
        self.sequence_idx =  '06'  #secuence
        self.for_idx=0
        self.curr_se3= np.eye(4)   #initial position
        self.curr_scan_pts = []
        self.prev_scan_pts =[]
        self.curr_scan_down_pts=[]
        self.prev_scan_down_pts=[]
        self.odom_transform=[]
        self.icp_initial = np.eye(4)
        self.pose_list = np.reshape(np.eye(4), (-1, 16))
        rospy.init_node (name, anonymous = True)
        rospy.Subscriber('/velodyne_points',PointCloud2,self.read_LiDAR)
        self.pub = rospy.Publisher('/Odom', Odometry, queue_size=10)
        rospy.loginfo("Starting Node")
        rospy.spin()
        
        
    def read_LiDAR(self,data):

        rate = rospy.Rate(20)
        self.num_frames = 6000

       
        
        # Result saver
        self.save_dir = "result/" + self.sequence_idx
        if not os.path.exists(self.save_dir): os.makedirs(self.save_dir)

        
       
        fig_idx = 1
       
        num_frames_to_skip_to_show = 5
        
                # get current information     
        self.curr_scan_pts= ptstoxyz.pointcloud2_to_xyz_array(data,remove_nans=True)
        print(self.curr_scan_pts.shape)
        
        self.curr_scan_down_pts = Ptutils.random_sampling(self.curr_scan_pts, num_points=self.num_icp_points)
    
            # save current node
        curr_node_idx = self.for_idx # make start with 0      
        if(curr_node_idx == 0):
            self.prev_node_idx = curr_node_idx
            self.prev_scan_pts = copy.deepcopy(self.curr_scan_pts)
            self.icp_initial = np.eye(4)
            
    
            # calc odometry

        self.prev_scan_down_pts = Ptutils.random_sampling(self.prev_scan_pts, num_points=self.num_icp_points)
        self.odom_transform, _, _ = ICP.icp(self.curr_scan_down_pts, self.prev_scan_down_pts, init_pose=self.icp_initial, max_iterations=20)
 
        self.curr_se3 = np.matmul(self.curr_se3, self.odom_transform)
        
        self.icp_initial = self.odom_transform # assumption: constant velocity model (for better next ICP converges)


        # renewal the prev information 
        self.prev_node_idx = curr_node_idx
        self.prev_scan_pts = copy.deepcopy(self.curr_scan_pts)

        

        # save the ICP odometry pose result 

        self.pose_list = np.vstack((self.pose_list, np.reshape(self.curr_se3 , (-1, 16))))
        if(curr_node_idx % self.save_gap == 0 or curr_node_idx == self.num_frames):        
            # save odometry-only poses
            filename = "pose" + self.sequence_idx + "unoptimized_" + str(int(time.time())) + ".csv"
            filename = os.path.join(self.save_dir, filename)
            np.savetxt(filename, self.pose_list, delimiter=",")
        
        
        if(self.for_idx % num_frames_to_skip_to_show == 0): 
            self.x = self.pose_list[:,3]
            self.y = self.pose_list[:,7]
            
            #print graph
            fig = plt.figure(fig_idx)
            plt.clf()
            plt.plot(-self.y, self.x, color='blue') 
            plt.axis('equal')
            plt.xlabel('x', labelpad=10) 
            plt.ylabel('y', labelpad=10)
            plt.draw()
            plt.pause(0.01) #is necessary for the plot to update for some reason

        self.for_idx= self.for_idx+1
        #odometry publisher
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = " UTM_COORDINATE "
        msg.pose.pose.position.x = -self.y[curr_node_idx-4]
        msg.pose.pose.position.y = self.x[curr_node_idx-4]
    	
         
        self.pub.publish(msg)
        rate.sleep()
         
       


if __name__ == '__main__':
    node_int = laser_subs('Laser_subs')
    
        