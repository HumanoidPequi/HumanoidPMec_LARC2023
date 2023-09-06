#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

going = True

def talker(angles):
    pub = rospy.Publisher('/servo', Int16MultiArray, queue_size=10)
    # If the motor has reached its limit, publish a new command.
    pub.publish(angles)

if __name__ == '__main__':
    rospy.init_node('teste_dimitria', anonymous=True)
    angles = Int16MultiArray()
    while not rospy.is_shutdown():
        try:
            if going:
                angles.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            else:
                angles.data = [0, 0, 0, 0, 0, 0, 20, 0, 0, 0, 0, 0]
            talker(angles)
            going = not going  # Inverte o valor de 'going' a cada iteração
        except rospy.ROSInterruptException:
            pass
        rospy.Rate(50).sleep()
