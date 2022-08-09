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
target = [0, 0, 5]

pub = None
data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
data_to_send2 = Float64MultiArray()  # the data to be sent, initialise the array


def output(message, erro, motor):

    # esta funcao centraliza todas as impressoes de algo na tela

    # print robot_pose

    for i in message:
        print (i, end=' '),        
    if erro:
        print (giro_helices, "erro: {:.2f}".format(
            erro), "altura: {:.2f}".format(last_pose[motor][2]))    
    else:
        print()
    


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
    Kp = 100
    Ki = 10
    Kd = 0

    global controller1
    controller1 = PID(Kp, Ki, Kd)
    controller1.send(None)

    global controller2 
    controller2 = PID(Kp, Ki, Kd)
    controller2.send(None)

    global controller3 
    controller3 = PID(Kp, Ki, Kd)
    controller3.send(None)

    global controller4 
    controller4 = PID(Kp, Ki, Kd)
    controller4.send(None)

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

    while not rospy.is_shutdown() and not roboVirado():

        giro_helices[0] = controller1.send([t, robot_pose[0][2], alvo])
        giro_helices[1] = controller2.send([t, robot_pose[1][2], alvo])
        giro_helices[2] = controller3.send([t, robot_pose[2][2], alvo])
        giro_helices[3] = controller4.send([t, robot_pose[3][2], alvo])

        output('Atualizando', alvo - robot_pose[0][2],0)
        output('Atualizando', alvo - robot_pose[1][2],1)
        output('Atualizando', alvo - robot_pose[2][2],2)
        output('Atualizando', alvo - robot_pose[3][2],3)

        data_to_send.data = [giro_helices[0], -giro_helices[1], -giro_helices[2], giro_helices[3]]
        
        pub.publish(data_to_send)
        pub2.publish(data_to_send2) # mantem a posicao

        last_pose = copy.deepcopy(robot_pose)

        # print robot_pose, giro_helices

        t = t + (1/frequencia)

        rospy.Rate(frequencia).sleep()

    rospy.spin()


if __name__ == '__main__':
    FlightController()
