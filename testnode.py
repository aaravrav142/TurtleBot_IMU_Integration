#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist,PoseStamped

def main():
    
    rospy.init_node('test')
    movement = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size =10)
    whereto = PoseStamped()
    whereto.header.frame_id = 'odom'

    rospy.sleep(1)
    #while not rospy.is_shutdown():
    x = 0
    while x < 4:
        whereto.header.stamp = rospy.Time.now()
        whereto.header.seq = 15
        whereto.pose.position.x = .5*x
        whereto.pose.position.y = .5*x
        quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
        whereto.pose.orientation.x = quaternion[0]
        whereto.pose.orientation.y = quaternion[1]
        whereto.pose.orientation.z = quaternion[2]
        whereto.pose.orientation.w = quaternion[3]

        movement.publish(whereto)
        x = x+1
        rospy.sleep(12)
        #ra = rospy.Rate(1)
        #ra.sleep()



if __name__ == '__main__':
    
    
    main()
