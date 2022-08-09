#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import LinkStates
import copy

frequencia = 5  # frequencia de atualizacao do controle

robot_pose = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
last_pose = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
giro_helices = [0, 0, 0, 0]
tilt_helices = [0, 0, 0, 0]
target = [0, 0, 1]

pub = None
data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
data_to_send2 = Float64MultiArray()  # the data to be sent, initialise the array


def output(message, erro, motor):

    # esta funcao centraliza todas as impressoes de algo na tela

       

    for i in message:
        print (i, ',', end=' ')
        print (giro_helices, robot_pose)        
    """
    if erro:
        print (giro_helices, "erro: {:.2f}".format(
            erro), "altura: {:.2f}".format(last_pose[motor][2]))    
    else:
        print()
    """


def updatePosition(data):

    # Esta funcao atualiza a posicao do robo na variavel global robot_pose

    global robot_pose

    odometry_index = data.name.index("robot::motor_front_right")

    robot_pose[0][0] = data.pose[odometry_index].position.x
    robot_pose[0][1] = data.pose[odometry_index].position.y
    robot_pose[0][2] = data.pose[odometry_index].position.z

    odometry_index = data.name.index("robot::motor_front_left")

    robot_pose[1][0] = data.pose[odometry_index].position.x
    robot_pose[1][1] = data.pose[odometry_index].position.y
    robot_pose[1][2] = data.pose[odometry_index].position.z

    odometry_index = data.name.index("robot::motor_back_right")

    robot_pose[2][0] = data.pose[odometry_index].position.x
    robot_pose[2][1] = data.pose[odometry_index].position.y
    robot_pose[2][2] = data.pose[odometry_index].position.z

    odometry_index = data.name.index("robot::motor_back_left")

    robot_pose[3][0] = data.pose[odometry_index].position.x
    robot_pose[3][1] = data.pose[odometry_index].position.y
    robot_pose[3][2] = data.pose[odometry_index].position.z

    odometry_index = data.name.index("robot::odom")

    robot_pose[4][0] = data.pose[odometry_index].position.x
    robot_pose[4][1] = data.pose[odometry_index].position.y
    robot_pose[4][2] = data.pose[odometry_index].position.z

def PID(Kp, Ki, Kd, MV_bar=0):
    # Kp = ganho proporcional
    # SP = SETPOINT, valor de referencia
    # MV = MANIPULATED VALUE, valor a ser enviado para o controle
    # PV = PROCESS VARIABLE, VALOR atual da variavel controlada

    # initialize stored data
    e_prev = 0
    t_prev = -100
    I = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, PV, SP = yield MV
        
        # PID calculations
        e = SP - PV
        
        P = Kp*e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)
        
        MV = MV_bar + P + I + D
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t

def demonstracao(giro_helices, t, alvo):
    d = 100
    fim = False

    giros = [giro_helices[0], -giro_helices[0], -giro_helices[0], giro_helices[0]]

    if t<=10:

        """
    if  t <= 1:       
        # girar Hélice 1            
        output(['Girando Helice 1'], False,0)  
        giros = [d, 0 , 0, 0]
    if t > 1 and t <= 2:
        # girar Hélice 2            
        output(['Girando Helice 2'], False,0)  
        giros = [0, -d , 0, 0]
    if t > 2 and t <= 3:
        # girar Hélice 3            
        output(['Girando Helice 3'], False,0)  
        giros = [0, 0 , -d, 0]
    if t > 3 and t <= 4:
        # girar Hélice 4            
        output(['Girando Helice 4'], False,0)  
        giros = [0, 0 , 0, d]
    if t > 4 and t <= 5:
        # Parar hélices           
        output(['Hélices desligadas'], False,0)  
        giros = [0,0,0,0]

    if t > 5 and t <= 10:
        """
        output([t, 'Planando a ' + str(alvo) + ' m'], alvo - robot_pose[4][2],0)
        giros = [giro_helices[0], -giro_helices[0], -giro_helices[0], giro_helices[0]]
        """
    if t > 10 and t <= 12:
        # girar para direita            
        output(['Girar para direita'], alvo - robot_pose[4][2],0)  
        giros = [giro_helices[0]+d, -(giro_helices[0]-d), -(giro_helices[0]-d), giro_helices[0]+d]
    if t > 12 and t <= 14:
        # girar para esquerda  
        output(['Girar para esquerda'], alvo - robot_pose[4][2],0)  
        giros = [giro_helices[0]-d, -(giro_helices[0]+d), -(giro_helices[0]+d), giro_helices[0]-d]
    if t > 14 and t <= 18:            
        # ir para frente 
        d = 1
        output(['Ir para frente'], alvo - robot_pose[4][2],0)  
        giros = [giro_helices[0]+d, -(giro_helices[0]+d), -(giro_helices[0]-d), giro_helices[0]-d]      
        """
    if t > 10 and t<= 12:
        alvo = 0
        output([t, 'Pouso'], alvo - robot_pose[4][2],0)  
        giros = [giro_helices[0], -giro_helices[0], -giro_helices[0], giro_helices[0]]       
    if t > 12:
        alvo = 0
        output([t, 'Hélices desligadas'], False,0)  
        giros = [0,0,0,0]
    if t > 13:   
        fim = True

    return giros, alvo, fim

def roboVirado():
    if robot_pose[4][2] > robot_pose[0][2] and robot_pose[4][2] > robot_pose[1][2] and robot_pose[4][2] > robot_pose[2][2] and robot_pose[4][2] > robot_pose[3][2]:
        output(["***** ROBO VIRADO ******"], False, 0)
        return True
    return False

def FlightController():

    # esta eh a funcao principal, que manda calcular a velocidade das helices e publica

    # iniciar os controladores proporcionais

    eixo = 2

    alvo = target[eixo]
    Kp = 500
    Ki = 50
    Kd = 500
    K = 1

    global controller1
    controller1 = PID(Kp, Ki, Kd)
    controller1.send(None)


    global pub
    global data_to_send
    global data_to_send2
    global chao
    global last_pose
    global robot_pose

    rospy.init_node('FlightController', anonymous=True)

    rospy.Subscriber('gazebo/link_states', LinkStates, updatePosition)

    pub = rospy.Publisher('/lophorina/lophorina_prop_controller/command',
                          Float64MultiArray, queue_size=1)
    pub2 = rospy.Publisher('/lophorina/lophorina_tilt_controller/command',
                          Float64MultiArray, queue_size=1)
    chao = robot_pose[4][2]    

    data_to_send2.data = [tilt_helices[0], tilt_helices[1], tilt_helices[2], tilt_helices[3]]

    pub2.publish(data_to_send2)

    # print 'Lets publish'

    t = 0
    fim = False

    while not rospy.is_shutdown() and not roboVirado() and not fim:

        giro_helices[0] = K * controller1.send([t, robot_pose[4][2], alvo])

        if giro_helices[0] < 0:
            giro_helices[0] = 0       

        data_to_send.data, alvo, fim = demonstracao(giro_helices, t, alvo)
        
        pub.publish(data_to_send)
        pub2.publish(data_to_send2) # mantem a posicao

        last_pose = copy.deepcopy(robot_pose)

        # print robot_pose, giro_helices

        t = t + (1/frequencia)

        rospy.Rate(frequencia).sleep()


    output(["***** FIM ******"], False, 0)

    rospy.spin()    


if __name__ == '__main__':    
    FlightController()
