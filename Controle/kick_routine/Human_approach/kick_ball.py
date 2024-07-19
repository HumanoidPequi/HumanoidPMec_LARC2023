
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class KickBall:
    def __init__(self):
        rospy.init_node('kick_ball', anonymous=True)
        
        # Publishers for joint controllers
        self.joint1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
        
        # Initialize the node
        self.rate = rospy.Rate(10) # 10hz
        self.joint_positions = {
            'joint1': 0.0,
            'joint2': 0.0
        }

        rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_callback)
        
        rospy.loginfo("KickBall node initialized")

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]
        
    def move_joint(self, joint, position):
        if joint == 'joint1':
            self.joint1_pub.publish(position)
        elif joint == 'joint2':
            self.joint2_pub.publish(position)

    def kick(self):
        rospy.loginfo("Preparing to kick the ball")
        
        # Example sequence to kick the ball
        self.move_joint('joint1', 0.5)  # Move joint1 to 0.5 radians
        self.rate.sleep()
        
        self.move_joint('joint2', 1.0)  # Move joint2 to 1.0 radians to kick
        self.rate.sleep()
        
        rospy.loginfo("Kick executed")
        
    def run(self):
        while not rospy.is_shutdown():
            self.kick()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        kicker = KickBall()
        kicker.run()
    except rospy.ROSInterruptException:
        pass
