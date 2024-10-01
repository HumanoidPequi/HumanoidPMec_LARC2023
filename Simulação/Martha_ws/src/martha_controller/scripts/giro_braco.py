#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import time

class GiroBraco:
    def __init__(self):
        # Inicializa o nó
        rospy.init_node('giro_braco', anonymous=True)
        
        # Define o publisher para o controlador de posição do ombro
        self.ombro_direito_pub = rospy.Publisher('/martha/ombro_direito_rX_position_controller/command', Float64, queue_size=10)
        
        # Frequência de publicação (Hz)
        self.rate = rospy.Rate(10)
        
    def run(self):
        angle = 0
        while not rospy.is_shutdown():
            # Converte o ângulo de graus para radianos
            angle_radians = math.radians(angle)
            
            # Cria a mensagem de ângulo
            angle_msg = Float64()
            angle_msg.data = angle_radians
            
            # Publica a mensagem no tópico
            self.ombro_direito_pub.publish(angle_msg)

            rospy.loginfo(angle)
            
            # Incrementa o ângulo
            angle = (angle + 1) % 360
            
            # Espera até a próxima iteração
            self.rate.sleep()

if __name__ == '__main__':
    try:
        giro_braco = GiroBraco()
        giro_braco.run()
    except rospy.ROSInterruptException:
        pass
