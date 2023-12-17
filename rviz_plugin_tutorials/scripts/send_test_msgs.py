#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from sensor_msgs.msg import Imu
import rospy
from math import cos, sin
import tf

topic = 'test_imu'
publisher = rospy.Publisher( topic, Imu, queue_size=5 )

rospy.init_node( 'test_imu' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
radius = 5
angle = 0
dir = 1
px = 0
py = 0
index = 0

dist = 3
while not rospy.is_shutdown():

    imu = Imu()
    imu.header.frame_id = "base_link"
    imu.header.stamp = rospy.Time.now()
    index+=1
    imu.header.seq = index
   
    imu.linear_acceleration.x = sin( 0 )
    imu.linear_acceleration.y = sin( angle )
    imu.linear_acceleration.z = sin( 10 )

    # 临时存储位置
    imu.angular_velocity.x = px
    imu.angular_velocity.y = py
    imu.angular_velocity.z = 0

    publisher.publish( imu )



    br.sendTransform((px, py, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    angle += dir * 0.01
    if angle >= 3.14: 
        dir = -3.14
    if angle < -3.14:
        dir = 3.14

    px = radius * cos(angle)
    py = radius * sin(angle)

    rate.sleep()

