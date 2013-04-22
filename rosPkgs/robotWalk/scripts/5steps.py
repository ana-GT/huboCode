#!/usr/bin/env python

import roslib; roslib.load_manifest('robotWalk')
import rospy, math

from std_msgs.msg import String

def demo():
    # Initialize the node
    print "Start demo"
    rospy.init_node('five_steps_demo')

    # Setup the publishers for each joint
    mode = rospy.Publisher('/atlas/mode', String, None, False, True, None)
    control_mode = rospy.Publisher('/atlas/control_mode', String, None, False, True, None)

    print "Before while"
    while mode.get_num_connections() == 0:
      rospy.sleep(0.1)

    print "After while"

    mode.publish("harnessed")
    control_mode.publish("stand-prep")
    print "Before sleep 5.0"
    rospy.sleep(5.0)
    mode.publish("nominal")
    rospy.sleep(0.3)
    control_mode.publish("stand")
    rospy.sleep(1.0)
#    control_mode.publish("walk")
    print "End demo"
if __name__ == '__main__':
    print "Start demo"
    try:
        demo()
    except rospy.ROSInterruptException: pass

