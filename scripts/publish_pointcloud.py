#!/usr/bin/env python

# Author: KhanhQuynhNguyen

import rospy, math
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import numpy as np


def read_numpy_file(file_link):
    data_storage=np.load(file_link)
    #print(data_storage)
    return data_storage

def publish_obstacle_msg():
    pub = rospy.Publisher('segmentation/data', PointCloud, queue_size=1)
    rospy.init_node("Segmenation_Data")
    
    segmentation_msgs=PointCloud() #The msg type is PointCloud
    rate=rospy.Rate(10) #10Hz publish
    raw_data=[]
    raw_data=read_numpy_file('/home/frauas/indoor_ws/src/BENg_thesis/scripts/final_sample.npy')
    index=0
        
    for point in raw_data:

      segmentation_msgs.points.append(Point32())
      segmentation_msgs.points[index].x=point[0]
      segmentation_msgs.points[index].y=point[1]
      segmentation_msgs.points[index].z=point[2]
      #rospy.loginfo(segmentation_msgs.points[index].x)
      index+=1
            
    while not rospy.is_shutdown():
        segmentation_msgs.header.stamp=rospy.Time.now()
        segmentation_msgs.header.frame_id="base_link"
        
        pub.publish(segmentation_msgs)   
        #del segmentation_msgs.points[:]
        rate.sleep()


if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass