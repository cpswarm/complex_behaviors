#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import rospkg
import tf
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

#definition of frame positions

#check if some robot is close to it and publish the transformation.
# subscribe to robot positions
# check if some robot is close and aligned with some QR
# publish the transform between the frame and the robot (same orientation)

#callbacks updating the position of the robots

#callbacks checking if the robot has a cart picked



# loop checking the position of the robots to publish the transforms.


#if has a cart dont publish the frame of the carts, only the door frame
#
# #publish frame with: ar_track_alvar_msgs/AlvarMarkers
# header: 
#  seq: 162
#  stamp: 
#    secs: 0
#    nsecs:         0
#  frame_id: ''
#markers: 
#  - 
#    header: 
#      seq: 0
#      stamp: 
#        secs: 3391
#        nsecs: 332000000
#      frame_id: "rb1_base_c_front_rgbd_camera_rgb_optical_frame"
#    id: 1
#    confidence: 0
#    pose: 
#      header: 
#        seq: 0
#        stamp: 
#          secs: 0
#          nsecs:         0
#        frame_id: ''
#      pose: 
#        position: 
#          x: -0.598308814619
#          y: -0.218161761489
#          z: 1.87468495538
#        orientation: 
#          x: 1.0
#          y: 0.0
#          z: -2.2964231681e-08
#          w: 6.30003754992e-08

#
# publish also by TF the frame


rb1_base_pose = Odometry()


rb1_base_ar_track_pub = rospy.Publisher('ar_pose_marker', AlvarMarkers, queue_size=10)

detection_radius = 3
cart_0_pose = Odometry()
cart_1_pose = Odometry()
cart_2_pose = Odometry()
cart_3_pose = Odometry()
cart_4_pose = Odometry()
cart_5_pose = Odometry()
cart_6_pose = Odometry()
cart_7_pose = Odometry()
cart_8_pose = Odometry()
cart_9_pose = Odometry()


def callback_pose_rb1_base(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global rb1_base_pose
    rb1_base_pose = data
   
#Callbacks to get poses of the carts
def callback_pose_cart_0(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_0_pose
    cart_0_pose = data
def callback_pose_cart_1(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_1_pose
    cart_1_pose = data
def callback_pose_cart_2(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_2_pose
    cart_2_pose = data
def callback_pose_cart_3(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_3_pose
    cart_3_pose = data
def callback_pose_cart_4(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_4_pose
    cart_4_pose = data
def callback_pose_cart_5(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_5_pose
    cart_5_pose = data
def callback_pose_cart_6(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_6_pose
    cart_6_pose = data
def callback_pose_cart_7(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_7_pose
    cart_7_pose = data
def callback_pose_cart_8(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_8_pose
    cart_8_pose = data
def callback_pose_cart_9(data):
    ##rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    global cart_9_pose
    cart_9_pose = data

def looking_at_marker0(pose):
    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_0_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_0_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 0 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_0():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_0 = AlvarMarker()
    marker_0.header.frame_id = "map"
    marker_0.header.stamp = rospy.Time.now()
    marker_0.id = 0
    marker_0.pose.pose.position.x = cart_0_pose.pose.pose.position.x
    marker_0.pose.pose.position.y = cart_0_pose.pose.pose.position.y
    marker_0.pose.pose.position.z = 1
    marker_0.pose.pose.orientation = cart_0_pose.pose.pose.orientation   
    markers.markers.append(marker_0)
    rb1_base_ar_track_pub.publish(markers)

    br1 = tf.TransformBroadcaster()
    #tr1 = Transform()
    #br1.sendTransform((marker_1.pose.pose.position.x, marker_1.pose.pose.position.y, 0), tf.transformations.quaternion_from_euler(0, math.pi/2, 0), rospy.Time.now(), "ar_marker_1", "rb1_base_a_front_rgbd_camera_rgb_optical_frame")
    br1.sendTransform((marker_0.pose.pose.position.x, marker_0.pose.pose.position.y, 0), (marker_0.pose.pose.orientation.x,marker_0.pose.pose.orientation.y,marker_0.pose.pose.orientation.z,marker_0.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_0", "map")

def looking_at_marker1(pose):
    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_1_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_1_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 1 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_1():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_1 = AlvarMarker()
    marker_1.header.frame_id = "map"
    marker_1.header.stamp = rospy.Time.now()
    marker_1.id = 1
    marker_1.pose.pose.position.x = cart_1_pose.pose.pose.position.x
    marker_1.pose.pose.position.y = cart_1_pose.pose.pose.position.y
    marker_1.pose.pose.position.z = 1
    marker_1.pose.pose.orientation = cart_1_pose.pose.pose.orientation
    markers.markers.append(marker_1)
    rb1_base_ar_track_pub.publish(markers)

    br1 = tf.TransformBroadcaster()
    #tr1 = Transform()
    #br1.sendTransform((marker_1.pose.pose.position.x, marker_1.pose.pose.position.y, 0), tf.transformations.quaternion_from_euler(0, math.pi/2, 0), rospy.Time.now(), "ar_marker_1", "rb1_base_a_front_rgbd_camera_rgb_optical_frame")
    br1.sendTransform((marker_1.pose.pose.position.x, marker_1.pose.pose.position.y, 0), (marker_1.pose.pose.orientation.x,marker_1.pose.pose.orientation.y,marker_1.pose.pose.orientation.z,marker_1.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_1", "map")


def looking_at_marker2(pose):
    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_2_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_2_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 2 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_2():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_2 = AlvarMarker()
    marker_2.header.frame_id = "map"
    marker_2.header.stamp = rospy.Time.now()
    marker_2.id = 2
    marker_2.pose.pose.position.x = cart_2_pose.pose.pose.position.x
    marker_2.pose.pose.position.y = cart_2_pose.pose.pose.position.y
    marker_2.pose.pose.position.z = 1
    marker_2.pose.pose.orientation = cart_2_pose.pose.pose.orientation
    markers.markers.append(marker_2)
    rb1_base_ar_track_pub.publish(markers)

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((marker_2.pose.pose.position.x, marker_2.pose.pose.position.y, 0), (marker_2.pose.pose.orientation.x,marker_2.pose.pose.orientation.y,marker_2.pose.pose.orientation.z,marker_2.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_2", "map")

def looking_at_marker3(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_3_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_3_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 3 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_3():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_3 = AlvarMarker()
    marker_3.header.frame_id = "map"
    marker_3.header.stamp = rospy.Time.now()
    marker_3.id = 3
    marker_3.pose.pose.position.x = cart_3_pose.pose.pose.position.x
    marker_3.pose.pose.position.y = cart_3_pose.pose.pose.position.x
    marker_3.pose.pose.position.z = 1
    marker_3.pose.pose.orientation = cart_3_pose.pose.pose.orientation
    markers.markers.append(marker_3)
    rb1_base_ar_track_pub.publish(markers)

    br3 = tf.TransformBroadcaster()
    br3.sendTransform((marker_3.pose.pose.position.x, marker_3.pose.pose.position.y, 0), (marker_3.pose.pose.orientation.x,marker_3.pose.pose.orientation.y,marker_3.pose.pose.orientation.z,marker_3.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_3", "map")



def looking_at_marker4(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_4_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_4_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 4 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_4():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_4 = AlvarMarker()
    marker_4.header.frame_id = "map"
    marker_4.header.stamp = rospy.Time.now()
    marker_4.id = 4
    marker_4.pose.pose.position.x = cart_4_pose.pose.pose.position.x
    marker_4.pose.pose.position.y = cart_4_pose.pose.pose.position.y
    marker_4.pose.pose.position.z = 1
    marker_4.pose.pose.orientation = cart_4_pose.pose.pose.orientation
    markers.markers.append(marker_4)
    rb1_base_ar_track_pub.publish(markers)

    br4 = tf.TransformBroadcaster()
    br4.sendTransform((marker_4.pose.pose.position.x, marker_4.pose.pose.position.y, 0), (marker_4.pose.pose.orientation.x,marker_4.pose.pose.orientation.y,marker_4.pose.pose.orientation.z,marker_4.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_4", "map")

def looking_at_marker5(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_5_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_5_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 5 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_5():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_5 = AlvarMarker()
    marker_5.header.frame_id = "map"
    marker_5.header.stamp = rospy.Time.now()
    marker_5.id = 5
    marker_5.pose.pose.position.x = cart_5_pose.pose.pose.position.x
    marker_5.pose.pose.position.y = cart_5_pose.pose.pose.position.y
    marker_5.pose.pose.position.z = 1
    marker_5.pose.pose.orientation = cart_5_pose.pose.pose.orientation
    markers.markers.append(marker_5)
    rb1_base_ar_track_pub.publish(markers)

    br5 = tf.TransformBroadcaster()
    br5.sendTransform((marker_5.pose.pose.position.x, marker_5.pose.pose.position.y, 0), (marker_5.pose.pose.orientation.x,marker_5.pose.pose.orientation.y,marker_5.pose.pose.orientation.z,marker_5.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_5", "map")
    
def looking_at_marker6(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_6_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_6_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 6 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_6():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_6 = AlvarMarker()
    marker_6.header.frame_id = "map"
    marker_6.header.stamp = rospy.Time.now()
    marker_6.id = 6
    marker_6.pose.pose.position.x = cart_6_pose.pose.pose.position.x
    marker_6.pose.pose.position.y = cart_6_pose.pose.pose.position.y
    marker_6.pose.pose.position.z = 1
    marker_6.pose.pose.orientation = cart_6_pose.pose.pose.orientation
    markers.markers.append(marker_6)
    rb1_base_ar_track_pub.publish(markers)

    br6 = tf.TransformBroadcaster()
    br6.sendTransform((marker_6.pose.pose.position.x, marker_6.pose.pose.position.y, 0), (marker_6.pose.pose.orientation.x,marker_6.pose.pose.orientation.y,marker_6.pose.pose.orientation.z,marker_6.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_6", "map")

def looking_at_marker7(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_7_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_7_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 7 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_7():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_7 = AlvarMarker()
    marker_7.header.frame_id = "map"
    marker_7.header.stamp = rospy.Time.now()
    marker_7.id = 7
    marker_7.pose.pose.position.x = cart_7_pose.pose.pose.position.x
    marker_7.pose.pose.position.y = cart_7_pose.pose.pose.position.y
    marker_7.pose.pose.position.z = 1
    marker_7.pose.pose.orientation = cart_7_pose.pose.pose.orientation
    markers.markers.append(marker_7)
    rb1_base_ar_track_pub.publish(markers)

    br7 = tf.TransformBroadcaster()
    br7.sendTransform((marker_7.pose.pose.position.x, marker_7.pose.pose.position.y, 0), (marker_7.pose.pose.orientation.x,marker_7.pose.pose.orientation.y,marker_7.pose.pose.orientation.z,marker_7.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_7", "map")

def looking_at_marker8(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_8_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_8_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 8 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_8():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_8 = AlvarMarker()
    marker_8.header.frame_id = "map"
    marker_8.header.stamp = rospy.Time.now()
    marker_8.id = 8
    marker_8.pose.pose.position.x = cart_8_pose.pose.pose.position.x
    marker_8.pose.pose.position.y = cart_8_pose.pose.pose.position.y
    marker_8.pose.pose.position.z = 1
    marker_8.pose.pose.orientation = cart_8_pose.pose.pose.orientation
    markers.markers.append(marker_8)
    rb1_base_ar_track_pub.publish(markers)

    br8 = tf.TransformBroadcaster()
    br8.sendTransform((marker_8.pose.pose.position.x, marker_8.pose.pose.position.y, 0), (marker_8.pose.pose.orientation.x,marker_8.pose.pose.orientation.y,marker_8.pose.pose.orientation.z,marker_8.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_8", "map")

def looking_at_marker9(pose):

    #Eucledian distance between cart and pose
    distance = math.sqrt(pow((pose.position.x - cart_9_pose.pose.pose.position.x ),2) + pow((pose.position.y - cart_9_pose.pose.pose.position.y),2))
    #rospy.loginfo("Distance from marker 9 %f", distance)
    if distance > detection_radius:
        return False
    return True

def publish_marker_9():
    global rb1_base_ar_track_pub
    markers = AlvarMarkers()
    markers.markers = []
    marker_9 = AlvarMarker()
    marker_9.header.frame_id = "map"
    marker_9.header.stamp = rospy.Time.now()
    marker_9.id = 9
    marker_9.pose.pose.position.x = cart_9_pose.pose.pose.position.x
    marker_9.pose.pose.position.y = cart_9_pose.pose.pose.position.y
    marker_9.pose.pose.position.z = 1
    marker_9.pose.pose.orientation = cart_9_pose.pose.pose.orientation
    markers.markers.append(marker_9)
    rb1_base_ar_track_pub.publish(markers)

    br9 = tf.TransformBroadcaster()
    br9.sendTransform((marker_9.pose.pose.position.x, marker_9.pose.pose.position.y, 0), (marker_9.pose.pose.orientation.x,marker_9.pose.pose.orientation.y,marker_9.pose.pose.orientation.z,marker_9.pose.pose.orientation.w), rospy.Time.now(), "ar_marker_9", "map")


def main():

    rospy.init_node("stage_frame_mapping_node")

    _name = rospy.get_name().replace('/','')

    rp = rospkg.RosPack()

    rospy.loginfo('%s: starting'%(_name))

    rate = rospy.Rate(100) # 100hz

    ## subscriptions

    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_pose_rb1_base)
    
    #subscription to all the cart poses
    
    rospy.Subscriber("/cart_0/base_pose_ground_truth", Odometry, callback_pose_cart_0)
    rospy.Subscriber("/cart_1/base_pose_ground_truth", Odometry, callback_pose_cart_1)
    rospy.Subscriber("/cart_2/base_pose_ground_truth", Odometry, callback_pose_cart_2)
    rospy.Subscriber("/cart_3/base_pose_ground_truth", Odometry, callback_pose_cart_3)
    rospy.Subscriber("/cart_4/base_pose_ground_truth", Odometry, callback_pose_cart_4)
    rospy.Subscriber("/cart_5/base_pose_ground_truth", Odometry, callback_pose_cart_5)
    rospy.Subscriber("/cart_6/base_pose_ground_truth", Odometry, callback_pose_cart_6)
    rospy.Subscriber("/cart_7/base_pose_ground_truth", Odometry, callback_pose_cart_7)
    rospy.Subscriber("/cart_8/base_pose_ground_truth", Odometry, callback_pose_cart_8)
    rospy.Subscriber("/cart_9/base_pose_ground_truth", Odometry, callback_pose_cart_9)
 
    while not rospy.is_shutdown():

	if looking_at_marker0(rb1_base_pose.pose.pose):
		publish_marker_0()

	if looking_at_marker1(rb1_base_pose.pose.pose):
		publish_marker_1()

	if looking_at_marker2(rb1_base_pose.pose.pose):
		publish_marker_2()    

	if looking_at_marker3(rb1_base_pose.pose.pose):
		publish_marker_3()

	if looking_at_marker4(rb1_base_pose.pose.pose):
		publish_marker_4()

	if looking_at_marker5(rb1_base_pose.pose.pose):
		publish_marker_5()

	if looking_at_marker6(rb1_base_pose.pose.pose):
		publish_marker_6()

	if looking_at_marker7(rb1_base_pose.pose.pose):
		publish_marker_7()

	if looking_at_marker8(rb1_base_pose.pose.pose):
		publish_marker_8()

	if looking_at_marker9(rb1_base_pose.pose.pose):
		publish_marker_9()

        rate.sleep()

if __name__ == "__main__":
    main()

