#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def set_joint_positions():
    rospy.init_node('darwin_op_fall_right', anonymous=True)

    # Publishers para as articulações do braço e pernas do Darwin-OP
    pub_r_shoulder_pitch = rospy.Publisher('/darwin/r_sho_pitch_position/command', Float64, queue_size=10)
    pub_r_shoulder_roll = rospy.Publisher('/darwin/r_sho_roll_position/command', Float64, queue_size=10)
    pub_r_elbow = rospy.Publisher('/darwin/r_el_position/command', Float64, queue_size=10)
    
    pub_l_shoulder_pitch = rospy.Publisher('/darwin/l_sho_pitch_position/command', Float64, queue_size=10)
    pub_l_shoulder_roll = rospy.Publisher('/darwin/l_sho_roll_position/command', Float64, queue_size=10)
    pub_l_elbow = rospy.Publisher('/darwin/l_el_position/command', Float64, queue_size=10)

    pub_r_hip_yaw = rospy.Publisher('/darwin/r_hip_yaw_position/command', Float64, queue_size=10)
    pub_r_hip_roll = rospy.Publisher('/darwin/r_hip_roll_position/command', Float64, queue_size=10)
    pub_r_hip_pitch = rospy.Publisher('/darwin/r_hip_pitch_position/command', Float64, queue_size=10)
    pub_r_knee = rospy.Publisher('/darwin/r_knee_position/command', Float64, queue_size=10)
    pub_r_ankle_pitch = rospy.Publisher('/darwin/r_ank_pitch_position/command', Float64, queue_size=10)
    pub_r_ankle_roll = rospy.Publisher('/darwin/r_ank_roll_position/command', Float64, queue_size=10)

    pub_l_hip_yaw = rospy.Publisher('/darwin/l_hip_yaw_position/command', Float64, queue_size=10)
    pub_l_hip_roll = rospy.Publisher('/darwin/l_hip_roll_position/command', Float64, queue_size=10)
    pub_l_hip_pitch = rospy.Publisher('/darwin/l_hip_pitch_position/command', Float64, queue_size=10)
    pub_l_knee = rospy.Publisher('/darwin/l_knee_position/command', Float64, queue_size=10)
    pub_l_ankle_pitch = rospy.Publisher('/darwin/l_ank_pitch_position/command', Float64, queue_size=10)
    pub_l_ankle_roll = rospy.Publisher('/darwin/l_ank_roll_position/command', Float64, queue_size=10)

    rospy.sleep(1)  # Dar tempo para os publishers inicializarem

    # Levanta o braço direito
    pub_r_shoulder_pitch.publish(1.5)  # Posição elevada
    pub_r_shoulder_roll.publish(-0.5)  # Para o lado direito
    pub_r_elbow.publish(-1.0)          # Dobra o cotovelo

    # Coloca o braço esquerdo para baixo
    pub_l_shoulder_pitch.publish(-1.5)
    pub_l_shoulder_roll.publish(0.5)
    pub_l_elbow.publish(0.0)

    # Mover as pernas para cair para o lado direito
    pub_r_hip_roll.publish(-0.5)
    pub_r_ankle_roll.publish(-0.5)

    pub_l_hip_roll.publish(0.5)
    pub_l_ankle_roll.publish(0.5)

    rospy.sleep(1)  # Espera para que as posições sejam atingidas

if __name__ == '__main__':
    try:
        set_joint_positions()
    except rospy.ROSInterruptException:
        pass
