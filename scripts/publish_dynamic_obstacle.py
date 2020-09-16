#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance, PointStamped, Point
from tf.transformations import quaternion_from_euler


def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  
  rospy.init_node("custom_obstacle_msg")
  tf_listener=tf.TransformListener()
  y_0 = -3.0
  vel_x = 0.0
  vel_y = 0.0
  range_y = 6.0


  #print(obstacle_msg)
  r = rospy.Rate(10) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
    
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

    #Convert point from base_link frame to map_frame
    pointstamp = PointStamped()
    pointstamp.header.frame_id = 'base_link'
    pointstamp.header.stamp = rospy.Time.now()
    pointstamp.point.x = 1.0
    pointstamp.point.y = 2.0
    pointstamp.point.z = 0
    converted_point=PointStamped()
    try:
      tf_listener.waitForTransform("/base_link", "/map", rospy.Time.now(), rospy.Duration(4.0))
      converted_point=tf_listener.transformPoint('/map', pointstamp)
    except tf.Exception:
      pass
    
    print(converted_point.point.x)
    
    # Add point obstacle
    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[0].id = 99
    obstacle_msg.obstacles[0].polygon.points = [Point32()]
    obstacle_msg.obstacles[0].polygon.points[0].x = converted_point.point.x
    obstacle_msg.obstacles[0].polygon.points[0].y = converted_point.point.y
    obstacle_msg.obstacles[0].polygon.points[0].z = 0

    yaw = math.atan2(vel_y, vel_x)
    q = tf.transformations.quaternion_from_euler(0,0,yaw)
    obstacle_msg.obstacles[0].orientation = Quaternion(*q)

    obstacle_msg.obstacles[0].velocities.twist.linear.x = 0
    obstacle_msg.obstacles[0].velocities.twist.linear.y = 0
    obstacle_msg.obstacles[0].velocities.twist.linear.z = 0
    obstacle_msg.obstacles[0].velocities.twist.angular.x = 0
    obstacle_msg.obstacles[0].velocities.twist.angular.y = 0
    obstacle_msg.obstacles[0].velocities.twist.angular.z = 0
    
    pub.publish(obstacle_msg)
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

