import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom
from geometry_msgs.msg import Twist


# Adicione aqui os imports necessários


pi = math.pi


# Controlar o robô para que ele saia do labirinto e pare do lado 
#   de fora.
# Deve ser capaz de sair independentemente da posição inicial 
#   do robô.
# Deve resolver o labirinto em menos de 5 minuto.

class BaseControlNode(Node,Laser, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('q1_node') 
        Laser.__init__(self)
        Odom.__init__(self)
        self.etapa=0
        self.timer = self.create_timer(0.15, self.control)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_state = 'anda'
        self.state_machine = {
            'anda': self.anda,
            'vira': self.vira,
            'stop': self.stop,
            'direcao':self.direcao,
        }
        self.tempo_inicial = self.get_clock().now().to_msg().sec
        self.twist = Twist()
        self.i=0
        self.start_yaw=None
        self.goal_yaw = 0.0
        self.direcao=0.0
        # Inicialização de variáveis
        self.twist = Twist()
        self.x_inicial=None
        self.ajeitada='ajeitado'
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
            # DONE - Criar um publisher para o tópico /watcher que publica mensagens 
            #   do tipo std_msgs.msg.String.
            # Ao iniciar, o nó deve publicar a mensagem start no tópico
            #    /watcher.
        self.watcher = self.create_publisher(String, "watcher",10)
#         self.create_publisher(Twist , ‘cmd_vel, 10)
            # Twist: tipo da mensagem que será publicada.
            # 'cmd_vel': nome do tópico que será publicado.
            # 10: tamanho da fila de mensagens. Este argumento é opcional e o valor padrão é 10.

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def direcao(self):
        if self.x_inicial==None:
            self.x_inicial=self.x

        
        if self.x_inicial<0:
            self.start_yaw=-pi
            starty=self.start_yaw

            cima=starty-pi/2
            direita=starty-pi
            esquerda=starty
            baixo=starty+pi/2

            self.caminho =[2.0,'direita',1.0,'esquerda',4.0,'direita',1.0,'direita',6.0,'esquerda',0.5,'stop']

        elif self.x_inicial>0:# self.x_inicial>0:
            print(self.yaw)
            self.start_yaw=pi
            
            starty=self.start_yaw
            cima=starty-pi/2
            direita=starty-pi
            esquerda=starty
            baixo=starty+pi/2
            print(cima)
            self.caminho =[cima,direita,cima,esquerda,cima,'stop']
        else:
            self.x_inicial=self.x
            self.start_yaw=self.yaw
            return
        self.goal_yaw=self.caminho[self.etapa]

        
        self.robot_state='vira'


        
        
    

    
    def anda(self):
        self.twist.linear.x = 0.3
        front_distance = min(self.front)
        
        if front_distance < 0.3:
            self.robot_state = 'direcao'
        
        if min(self.right)<0.3:
            self.ajeitada='esquerda'
            self.state_machine='vira'

        elif min(self.left)<0.3:
            self.ajeitada='direita'
            self.state_machine='vira'

        

    def vira(self):
        if self.ajeitada=='esquerda':
            self.goal_yaw +=pi/10
        elif self.ajeitada=='direita':
            self.goal_yaw-=pi/10
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        print(self.goal_yaw)
        if np.abs(erro) < np.deg2rad(1):
            if self.ajeitada!='ajeitado':
                self.etapa+=1
            self.ajeitada!='ajeitado'
            self.robot_state='anda'
            self.tempo_inicial = self.get_clock().now().to_msg().sec
        else:
            if erro < 0:
                self.twist.angular.z = -1*abs(erro)  # sentido horário
                # print(f'vel horario:{self.twist.angular.z}') 
            else:
                self.twist.angular.z = 1*abs(erro)
                # print(f'vel anti-horario:{self.twist.angular.z}')  # sentido anti-horário
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        # print(f'etapa Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        front_distance = min(self.front)
        

        


        self.cmd_vel_pub.publish(self.twist)
        self.i=1
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseControlNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()