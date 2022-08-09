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
giro_minimo_subida = [5084, 5084, 5084, 5084]
giro_helices_max = [20000,  20000,  20000,  20000]
target_max = [0, 0, 5.1]
target = [0, 0, 5]
target_min = [0, 0, 4.9]
step = [10, 10, 10]

pub = None
data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
data_to_send2 = Float64MultiArray()  # the data to be sent, initialise the array


def output(message, velocidade, motor):
    global chao

    # esta funcao centraliza todas as impressoes de algo na tela

    # print robot_pose

    for i in message:
        print (i, end=' '),        
    if velocidade:
        print (giro_helices, "vel de subida: {:.2f}".format(
            velocidade), "altura: {:.2f}".format(last_pose[motor][2]))    
    else:
        print()
    if robot_pose[4][2] <= chao:
            print ('No chao')
    


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


def getStep(motor, eixo):

    # o step eh uma porcentagem do erro

    tax = 1

    return abs(getError(motor, eixo)) * tax


def getError(motor, eixo):

    # Esta funcao busca a diferenca entre o valor desejado e o verdadeiro no eixo (x,y ou z) e motor selecionado

    if robot_pose[motor][eixo] >= target_min[eixo] and robot_pose[motor][eixo] <= target_max[eixo]:
        return 0

    return robot_pose[motor][eixo] - target[eixo]

def getVelocidadeNoEixo(motor, eixo):

    return robot_pose[motor][eixo] - last_pose[motor][eixo]


def calcVel(eixo, direction):

    # esta funcao determina qual sera a velociade das helices
    # o eixo indica qual o eixo a ser considerado: x,y,z = 0,1,2
    # direction indica se o robo esta tentado subir ou descer no eixo z
    # isso eh usado para evitar que ajustes dos outros eixos facam ele ir
    # para o lado errado no eixo z

    global giro_helices
    global tilt_helices
    global last_pose
    global giro_minimo_subida

    global state

    retorno = 0   


    if eixo == 2:  # eixo z

        for motor in range(4):

            erro = getError(motor, eixo)
            velocidadeNoEixo = getVelocidadeNoEixo(motor, eixo)
            stepSubida = 5
            stepDescida = 13
            velocidadeMaximaSubida = 0.2
            velocidadeMaximaDescida = 1
            
            if erro == 0: 

                output(["Eixo z OK"], velocidadeNoEixo, motor)

                if velocidadeNoEixo > 0 and giro_helices[motor] >= giro_minimo_subida[motor]:
                    giro_helices[motor] = giro_helices[motor] - 1
                if velocidadeNoEixo < 0:
                    if giro_helices[motor] < giro_minimo_subida[motor]:
                        giro_helices[motor] = giro_minimo_subida[motor]
                    giro_helices[motor] = giro_helices[motor] + stepSubida

                tilt_helices[motor] = 0
                state[motor] = ['OK', erro]

            if erro < 0:  # deve subir

                if velocidadeNoEixo <= 0 : # esta caindo

                    tilt_helices[motor] = 0
                    
                    if giro_helices[motor] < giro_minimo_subida[motor]:
                        output(["giro minimo em ", motor ], False, motor)
                        giro_helices[motor] = giro_minimo_subida[motor]
                    else:
                        #verificar se o motor esta estabilizado
                        if robot_pose[motor][eixo] <= robot_pose[4][eixo] + 0.026:
                            output(["aumentando giro em ", motor], velocidadeNoEixo, motor)
                            giro_helices[motor] = giro_helices[motor] + stepSubida
                        else:
                            output(["estabilizando giro em ", motor], velocidadeNoEixo, motor)

                else: # esta subindo
                    if state[motor][0] == 'DECOLAGEM': # atualizar giro minimo e manter giro

                        output(["UP giro minimo salvo em ", motor], velocidadeNoEixo, motor)
                        giro_minimo_subida[motor] = giro_helices[motor]


                    else:
                        
                        if giro_helices[motor] > giro_minimo_subida[motor] and velocidadeNoEixo > velocidadeMaximaSubida:
                            output(["UP limitando subida em ", motor], velocidadeNoEixo, motor)
                            giro_helices[motor] = giro_minimo_subida[motor]

                            state[motor] = ['SUBINDO', erro]

                        else:
                            output(["UP mantendo giro em ", motor], velocidadeNoEixo, motor)
                            giro_helices[motor] = giro_minimo_subida[motor]                     
                            # giro_helices = [x - (stepSubida / 5) for x in giro_helices]
                            state[motor] = ['SUBINDO', erro]
                        
                        

                    
                    

            if erro > 0:  # deve descer            

                if velocidadeNoEixo > 0 : # esta subindo
                    if robot_pose[motor][eixo] - 0.025 >= robot_pose[4][eixo]:
                        output(["dw em ", motor], velocidadeNoEixo, motor)
                        giro_helices[motor] = giro_helices[motor] - stepDescida
                    else:
                        output(["estabilizando giro em ", motor], velocidadeNoEixo, motor)

                else: # esta caindo
                    output(["DW em ", motor], velocidadeNoEixo, motor)
                    state = ['DESCENDO', erro]
                    if giro_helices[motor] < giro_minimo_subida[motor] and velocidadeNoEixo < velocidadeMaximaDescida:
                        giro_helices[motor] = giro_helices[motor] + stepSubida
                    else:
                        giro_helices[motor] = giro_minimo_subida[motor]
                        #giro_helices = [x + (stepDescida/10) for x in giro_helices]
                    


    if eixo == 1:  # eixo y
        output(["Eixo y"], False , 0)

    if eixo == 0:  # eixo x
        output(["Eixo x"], False, 0)



    last_pose = copy.deepcopy(robot_pose)

    return retorno


def roboVirado():
    if robot_pose[4][2] > robot_pose[0][2] and robot_pose[4][2] > robot_pose[1][2] and robot_pose[4][2] > robot_pose[2][2] and robot_pose[4][2] > robot_pose[3][2]:
        output(["***** ROBO VIRADO ******"], False, 0)
        return True
    return False

def FlightController():

    # esta eh a funcao principal, que manda calcular a velocidade das helices e publica

    global pub
    global data_to_send
    global data_to_send2
    global state
    global chao

    state = [['DECOLAGEM'],['DECOLAGEM'],['DECOLAGEM'],['DECOLAGEM']]

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

    while not rospy.is_shutdown() and not roboVirado():

        direcao = calcVel(2, 0)
        # calcVel(0, direcao)
        # calcVel(1, direcao)
        data_to_send.data = [giro_helices[0], -giro_helices[1], -giro_helices[2], giro_helices[3]]
        
        pub.publish(data_to_send)

        # print robot_pose, giro_helices

        rospy.Rate(frequencia).sleep()

    rospy.spin()


if __name__ == '__main__':
    FlightController()
