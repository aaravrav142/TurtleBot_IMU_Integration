#!/usr/bin/env python  

import roslib
import rospy
import tf
import math
from geometry_msgs.msg import Twist,PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty,UInt8
from nav_msgs.msg import Odometry
from overhead_mobile_tracker.srv import *


def callback(data,args):

    pubodom,pubfiltered,listener,trans,rot,serv = args

    if data.data == 0:
        print "returned 0"
        #### Send empty message to reset_odometry topic to zero /odom
        empty = Empty()
        pubodom.publish(empty)

        #### Send 0 pose to /set_pose topic to /zero odometry/filtered
        newodom = PoseWithCovarianceStamped()
        newodom.header.frame_id = 'odom'
        newodom.header.stamp = rospy.Time.now()
        newodom.pose.pose.position.x = 0.0
        newodom.pose.pose.position.y = 0.0
        newodom.pose.pose.position.z = 0.0
        newodom.pose.pose.orientation.x = 0.0
        newodom.pose.pose.orientation.y = 0.0
        newodom.pose.pose.orientation.z = 0.0
        newodom.pose.pose.orientation.w = 1.0
        pubfiltered.publish(newodom)


        #### get current transform from base_meas to odom_meas


        try:
            (trans,rot) = listener.lookupTransform('/odom_meas','/base_meas',rospy.Time(0))
            print trans
            print rot
        except:
            print "transform not found"

        offset = Odometry()
        xsave = offset.pose.pose.position.x = -trans[0]
        ysave = offset.pose.pose.position.y = -trans[1]
        zsave = offset.pose.pose.position.z = -trans[2]

        x2 = offset.pose.pose.orientation.x = 0.0
        y2 = offset.pose.pose.orientation.y = 0.0
        z2 = offset.pose.pose.orientation.z = 0.0
        w2 = offset.pose.pose.orientation.w = 0.0
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion((x2,y2,z2,w2))

        #### use current transform to set odom_offset
        bool = serv(offset)
        print bool
        print "---------------------------------"

        try:
            (trans,rot) = listener.lookupTransform('/base_meas','/odom_meas',rospy.Time(0))
            print trans
            print rot
        except:
            print "transform not found"

        offset2 = Odometry()

        offset2.pose.pose.position.x = xsave
        offset2.pose.pose.position.y = ysave
        offset2.pose.pose.position.z = zsave

        x2 = offset2.pose.pose.orientation.x = rot[0]
        y2 = offset2.pose.pose.orientation.y = rot[1]
        z2 = offset2.pose.pose.orientation.z = rot[2]
        w2 = offset2.pose.pose.orientation.w = rot[3]

        bool = serv(offset2)
        print bool, "step 1"


    if data.data == 99:
        print "returned 99"
        empty = Empty()
        pubodom.publish(empty)

        #### Send 0 pose to /set_pose topic to /zero odometry/filtered
        newodom = PoseWithCovarianceStamped()
        newodom.header.frame_id = 'odom'
        newodom.header.stamp = rospy.Time.now()
        newodom.pose.pose.position.x = 0.0
        newodom.pose.pose.position.y = 0.0
        newodom.pose.pose.position.z = 0.0
        newodom.pose.pose.orientation.x = 0.0
        newodom.pose.pose.orientation.y = 0.0
        newodom.pose.pose.orientation.z = 0.0
        newodom.pose.pose.orientation.w = 1.0
        pubfiltered.publish(newodom)

 #### get current transform from base_meas to odom_meas


        try:
            (trans,rot) = listener.lookupTransform('/odom_meas','/base_meas',rospy.Time(0))
            print trans
            print rot
        except:
            print "transform not found"
        offset = Odometry()
        xsave = offset.pose.pose.position.x = -trans[0]
        ysave = offset.pose.pose.position.y = -trans[1]
        zsave = offset.pose.pose.position.z = -trans[2]

        x2 = offset.pose.pose.orientation.x = 0.0
        y2 = offset.pose.pose.orientation.y = 0.0
        z2 = offset.pose.pose.orientation.z = 0.0
        w2 = offset.pose.pose.orientation.w = 0.0
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion((x2,y2,z2,w2))

        #### use current transform to set odom_offset
        try:
            (trans,rot) = listener.lookupTransform('/base_meas','/odom_meas',rospy.Time(0))
            print trans
            print rot
        except:
            print "transform not found"

        offset2 = Odometry()

        offset2.pose.pose.position.x = xsave
        offset2.pose.pose.position.y = ysave
        offset2.pose.pose.position.z = zsave

        x2 = offset2.pose.pose.orientation.x = 0
        y2 = offset2.pose.pose.orientation.y = 0
        z2 = offset2.pose.pose.orientation.z = 0
        w2 = offset2.pose.pose.orientation.w = 0

        bool = serv(offset2)
        print bool, "step 1"


        

    if data.data == 10:
        print "returned 10"
        zero = Odometry()
        zero.pose.pose.position.x = 0.0
        zero.pose.pose.position.y = 0.0
        zero.pose.pose.position.z = 0.0

        x2 = zero.pose.pose.orientation.x = 0.0
        y2 = zero.pose.pose.orientation.y = 0.0
        z2 = zero.pose.pose.orientation.z = 0.0
        w2 = zero.pose.pose.orientation.w = 1.0

        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion((x2,y2,z2,w2))
        #print zero
        bool = serv(zero)
        
        print bool
        print "----------------------------"
        

def main():
    trans= 0
    rot = 0
    rospy.init_node('odom_sync')
    listener = tf.TransformListener()
    pubfiltered = rospy.Publisher('set_pose',PoseWithCovarianceStamped,queue_size = 10)
    pubodom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
    serv = rospy.ServiceProxy("set_offset",SetOdomOffset)
    rospy.Subscriber('/command_input',UInt8,callback,callback_args=(pubodom,pubfiltered,listener,trans,rot,serv))
    rospy.sleep(1)
    print "done sleeping"
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_meas', '/odom_meas',rospy.Time(0))

        except: continue


    rospy.spin()




if __name__ == '__main__':


    main()
