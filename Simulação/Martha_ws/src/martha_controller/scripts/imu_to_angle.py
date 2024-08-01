#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
import time

class MarthaController:
    def __init__(self):
        # Inicializa o nó
        rospy.init_node('martha_controller')
        
        # Define o publisher para os controladores de posição
        self.quadril_direito_pub = rospy.Publisher('/martha/quadril_direito_rX_position_controller/command', Float64, queue_size=10)
        self.quadril_esquerdo_pub = rospy.Publisher('/martha/quadril_esquerdo_rX_position_controller/command', Float64, queue_size=10)
        
        # Define o subscriber para o tópico IMU
        rospy.Subscriber('/martha/imu', Imu, self.imu_callback)
        
        # Constantes do controlador PID
        self.kp = 1
        self.kd = 0.001
        self.ki = 0.00001
        
        
        # Variáveis do controlador PID
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
    def imu_callback(self, data):
        # Converte quaternion para ângulos de Euler
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        
        # Erro em x (assumindo que queremos que roll seja 0)
        error_x = orientation_q.x
        
        # Tempo atual e delta de tempo
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Termo proporcional
        p_term = self.kp * error_x
        
        # Termo integral
        self.integral += error_x * dt
        i_term = self.ki * self.integral
        
        # Termo derivativo
        derivative = (error_x - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Esforço de controle total
        control_effort = -(p_term + i_term + d_term)
        
        # Atualiza valores para o próximo ciclo
        self.prev_error = error_x
        self.prev_time = current_time
        
        # Cria a mensagem para os controladores de posição
        angle_msg = Float64()
        angle_msg.data = -control_effort
        
        # Publica o controle nos dois controladores de quadril
        self.quadril_direito_pub.publish(angle_msg)
        self.quadril_esquerdo_pub.publish(angle_msg)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = MarthaController()
    controller.run()
