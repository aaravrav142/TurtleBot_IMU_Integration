#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist,PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty

def main():
    setodom()
    #setodomfiltered()

def setodom():
    rospy.init_node('test')
    empty = Empty()
    reset = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
    rospy.sleep(1)
    reset.publish(empty)


def setodomfiltered():

    #this resets the odometry/filtered value to the origin of the odom frame
    rospy.init_node('test')

    setodom = rospy.Publisher('set_pose',PoseWithCovarianceStamped,queue_size = 10)
    newodom = PoseWithCovarianceStamped()
    newodom.header.frame_id = 'odom'
    rospy.sleep(1)
    newodom.header.stamp = rospy.Time.now()
    newodom.pose.pose.position.x = 0.0
    newodom.pose.pose.position.y = 0.0
    newodom.pose.pose.position.z = 0.0

    #newodom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

    setodom.publish(newodom)

def movebot():
    
    rospy.init_node('test')
    movement = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size =10)

    whereto = PoseStamped()
    whereto.header.frame_id = 'odom'

    rospy.sleep(1)
    #while not rospy.is_shutdown():
    #x = 0
    #while x < 4:
    whereto.header.stamp = rospy.Time.now()
    whereto.header.seq = 15
    whereto.pose.position.x = -.5
    whereto.pose.position.y = 0
    quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
    whereto.pose.orientation.x = quaternion[0]
    whereto.pose.orientation.y = quaternion[1]
    whereto.pose.orientation.z = quaternion[2]
    whereto.pose.orientation.w = quaternion[3]

    movement.publish(whereto)
    #x = x+1
    #rospy.sleep(12)
    #ra = rospy.Rate(1)
    #ra.sleep()



if __name__ == '__main__':
    
    
    main()
