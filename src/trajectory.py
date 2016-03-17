#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist,PoseStamped,Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,UInt8
from sensor_msgs.msg import Imu


def callback(data,pub):
    t = Twist()
    
    if data.data == 1:
        t.linear.x = .25
        print t
    if data.data == 2:
        t.angular.z = 2
    if data.data == 3:
        t.linear.x = .25
        t.angular.z = 1

    time = rospy.get_time()
    while rospy.get_time()-time < 6:
        pub.publish (t)
        rospy.sleep(.005)




    



def main():
    rospy.init_node('trajectory')
    pub = rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size = 10)
    
    rospy.Subscriber('/command_input',UInt8,callback,callback_args=pub)
    
    rospy.spin()



if __name__ == '__main__':
    
    
    main()
