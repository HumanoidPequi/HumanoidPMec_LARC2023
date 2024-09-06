################### STILL UNTESTED ###################

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class KickBall:
    def __init__(self):
        rospy.init_node('kick_ball', anonymous=True)

        # Subscriber for kick command
        self.kick_sub = rospy.Subscriber('/kick_command', Bool, self.kick_callback)

        # Publisher for joint trajectories
        self.joint_traj_pub = rospy.Publisher('/martha/joint_trajectory_controller/command', JointTrajectory, queue_size=10)

    def kick_callback(self, data):
        if data.data:
            self.execute_kick()

    def send_joint_trajectory(self, joint_names, positions, time_from_start):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(time_from_start)
        traj.points.append(point)
        self.joint_traj_pub.publish(traj)

    def execute_kick(self):
        rospy.loginfo("Executing kick sequence...")

        # Names of the leg joints for kicking
        joint_names = ['quadril_esquerdo_rZ', 'quadril_esquerdo_rY', 'joelho_esquerdo_rX', 'tornozelo_esquerdo_rX', 'tornozelo_esquerdo_rY']

        # Step 1: Move leg back (recoil)
        positions = [-0.3, -0.4, 0.4, 0.2, -0.1]
        self.send_joint_trajectory(joint_names, positions, 1.0)
        rospy.sleep(1.0)

        # Step 2: Kick forward
        positions = [0.2, 0.6, -0.3, -0.2, 0.0]
        self.send_joint_trajectory(joint_names, positions, 0.5)
        rospy.sleep(0.5)

        # Step 3: Return to initial position
        positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joint_trajectory(joint_names, positions, 1.0)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        KickBall()
    except rospy.ROSInterruptException:
        pass
