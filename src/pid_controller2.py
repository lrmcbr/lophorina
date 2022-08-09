#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import LinkStates
import copy
import math

frequencia = 200  # frequencia de atualizacao do controle

robot_pose = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0,0,0,0]]
last_pose = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
giro_helices = [0, 0, 0, 0]
tilt_helices = [0, 0, 0, 0]
target = [0, 0, 1]

pub = None
data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
data_to_send2 = Float64MultiArray()  # the data to be sent, initialise the array


def output(t, tarefa):

    # esta funcao centraliza todas as impressoes de algo na tela

    if t==0:
        print('t, tarefa, giro0, giro1, giro2, giro3, x, y, z, r, p, y')

    print (t, ',', tarefa, ',', end=' ')
    print (', '.join([str(i) for i in giro_helices]), ', ', ', '.join([str(i) for i in robot_pose[4]]))


 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


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
    o = data.pose[odometry_index].orientation
    robot_pose[4][3], robot_pose[4][4], robot_pose[4][5] = euler_from_quaternion(o.x,o.y,o.z,o.w)

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

    tilt_helices = [0, 0, 0, 0]
    
    if t <= 5:
        output(t, 'Planando a ' + str(alvo) + ' m')
        giros = [giro_helices[0], -giro_helices[0], -giro_helices[0], giro_helices[0]]
    if t > 5:   
        g = 0.08727 #rad 
        tilt_helices = [g,-g,g,-g]
        output(t, 'Girando')

    return giros, alvo, fim, tilt_helices

def roboVirado(t):
    if robot_pose[4][2] > robot_pose[0][2] and robot_pose[4][2] > robot_pose[1][2] and robot_pose[4][2] > robot_pose[2][2] and robot_pose[4][2] > robot_pose[3][2]:
        output(t, "***** ROBO VIRADO ******")
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

    global controller1
    controller1 = PID(Kp, Ki, Kd)
    controller1.send(None)


    global pub
    global data_to_send
    global data_to_send2
    global last_pose
    global robot_pose

    rospy.init_node('FlightController', anonymous=True)

    rospy.Subscriber('gazebo/link_states', LinkStates, updatePosition)

    pub = rospy.Publisher('/lophorina/lophorina_prop_controller/command',
                          Float64MultiArray, queue_size=1)
    pub2 = rospy.Publisher('/lophorina/lophorina_tilt_controller/command',
                          Float64MultiArray, queue_size=1)

    data_to_send2.data = [tilt_helices[0], tilt_helices[1], tilt_helices[2], tilt_helices[3]]

    pub2.publish(data_to_send2)

    # print 'Lets publish'

    t = 0
    fim = False

    while not rospy.is_shutdown() and not roboVirado(t) and not fim:

        giro_helices[0] = controller1.send([t, robot_pose[4][2], alvo])

        if giro_helices[0] < 0:
            giro_helices[0] = 0       

        data_to_send.data, alvo, fim, data_to_send2.data  = demonstracao(giro_helices, t, alvo)
        
        pub.publish(data_to_send)
        pub2.publish(data_to_send2) # mantem a posicao

        last_pose = copy.deepcopy(robot_pose)

        # print robot_pose, giro_helices

        t = t + (1/frequencia)

        rospy.Rate(frequencia).sleep()


    output(t, "***** FIM ******")

    # rospy.spin()    


if __name__ == '__main__':    
    FlightController()
