#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

def get_cmd():
    cmd = Twist()
    ch = raw_input('Please use hjkl to steer the turtle: ')
    print 'You entered: ', ch
    if ch=='h':
        cmd.angular.z = 1.5
    if ch=='l':
        cmd.angular.z = -1.5
    if ch=='k':
        cmd.linear.x = 2
    if ch=='j':
        cmd.linear.x = -2
    return cmd


def teleop():
    topic = rospy.get_param('turtle_topicname', '/turtle1')
    pub = rospy.Publisher(topic + '/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_teleop')
    turtle_cmd = None
    while not rospy.is_shutdown():
        turtle_cmd = get_cmd()
        if not turtle_cmd == None:
            pub.publish(turtle_cmd)
        turtle_cmd = None

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
