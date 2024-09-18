################### UNTESTED ###################

#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Bool
import math

class KickController:
    def __init__(self):
        rospy.init_node('kick_controller', anonymous=True)

        # Subscrição para o comando de chute
        self.kick_sub = rospy.Subscriber('/kick_command', Bool, self.kick_callback)
        self.pub = rospy.Publisher('/humanoid/joint_trajectory_controller/command', JointTrajectory, queue_size=10)

        # Comprimentos aproximados da coxa e da perna, com base no modelo
        self.thigh_length = 0.4  # Comprimento da coxa em metros (aproximado)
        self.shin_length = 0.4   # Comprimento da perna em metros (aproximado)

        # Definir as juntas reais usadas para o movimento de chute
        self.joint_names = ['quadril_esquerdo_rX', 'joelho_esquerdo_rX', 'tornozelo_esquerdo_rX']

    def kick_callback(self, data):
        if data.data:
            # Posição alvo desejada para o pé ao chutar a bola (x, y)
            target_position = [0.3, -0.5]  # Exemplo de coordenadas alvo para o pé
            joint_angles = self.inverse_kinematics(target_position)
            self.perform_kick(joint_angles)

    def inverse_kinematics(self, target_position):
        x, y = target_position

        # Calcular a distância até o ponto alvo
        distance = math.sqrt(x**2 + y**2)

        # Verificar se o alvo está dentro do alcance da perna
        if distance > (self.thigh_length + self.shin_length):
            raise ValueError("Target position is out of reach")

        # Lei dos Cossenos para calcular o ângulo do joelho
        cos_knee_angle = (x**2 + y**2 - self.thigh_length**2 - self.shin_length**2) / (2 * self.thigh_length * self.shin_length)
        knee_angle = math.acos(cos_knee_angle)

        # Lei dos Cossenos para calcular o ângulo do quadril
        alpha = math.atan2(y, x)
        cos_hip_angle = (x**2 + y**2 + self.thigh_length**2 - self.shin_length**2) / (2 * self.thigh_length * distance)
        hip_angle = alpha - math.acos(cos_hip_angle)

        # Ângulo do tornozelo para ajustar a orientação do pé
        ankle_angle = -(hip_angle + knee_angle)

        return [hip_angle, knee_angle, ankle_angle]

    def move_to_position(self, joint_angles, duration):
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(duration)

        trajectory.points = [point]
        self.pub.publish(trajectory)

    def perform_kick(self, joint_angles):
        rospy.loginfo("Starting kick sequence")

        # Posição de equilíbrio inicial
        balance_positions = [0.0, 0.0, 0.0]
        self.move_to_position(balance_positions, 1.0)
        rospy.sleep(1.0)

        # Executar o chute com os ângulos calculados
        self.move_to_position(joint_angles, 1.0)
        rospy.sleep(1.0)

        # Retornar à posição inicial após o chute
        self.move_to_position(balance_positions, 1.0)
        rospy.sleep(1.0)

        rospy.loginfo("Kick sequence complete")
