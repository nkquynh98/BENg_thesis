#!/usr/bin/env python

# Author: KhanhQuynhNguyen

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, PointStamped, Point
from tf.transformations import quaternion_from_euler
import numpy as np

raw_data=[]

def read_numpy_file(file_link):
    data_storage=np.load(file_link)
    print(data_storage)
    return data_storage

def publish_obstacle_msg():
    pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    rospy.init_node("custom_obstacle_msg")

    y_0 = 0
    vel_x = 0.0
    vel_y = 0.0
    range_y = 6.0

    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
    
    # Add point obstacle
    index=0
    for point in raw_data[:100]:
        if point[2]==2:
            #print(point)
            obstacle_msg.obstacles.append(ObstacleMsg())
            obstacle_msg.obstacles[index].id = 99+index
            obstacle_msg.obstacles[index].polygon.points = [Point32()]
            obstacle_msg.obstacles[index].polygon.points[0].x = point[0]
            obstacle_msg.obstacles[index].polygon.points[0].y = point[1]
            obstacle_msg.obstacles[index].polygon.points[0].z = 0

            yaw = math.atan2(vel_y, vel_x)
            
            q = tf.transformations.quaternion_from_euler(0,0,yaw)
            obstacle_msg.obstacles[index].orientation = Quaternion(*q)

            obstacle_msg.obstacles[index].velocities.twist.linear.x = vel_x
            obstacle_msg.obstacles[index].velocities.twist.linear.y = vel_y
            obstacle_msg.obstacles[index].velocities.twist.linear.z = 0
            obstacle_msg.obstacles[index].velocities.twist.angular.x = 0
            obstacle_msg.obstacles[index].velocities.twist.angular.y = 0
            obstacle_msg.obstacles[index].velocities.twist.angular.z = 0
            index+=1
        
    print(obstacle_msg)
    r = rospy.Rate(10) # 10hz
    t = 0.0
    
    while not rospy.is_shutdown():


        pub.publish(obstacle_msg)

        r.sleep()

if __name__ == '__main__': 
  try:
    raw_data=read_numpy_file('/home/nkquynh/indoor_ws/src/indoor_thesis/scripts/final_sample.npy')
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

    