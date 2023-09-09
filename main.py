############### 10 minutos minimo #################################################################
# region MODULES
import matplotlib.pyplot as plt
try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')
import initializer as vrep_obj
import Trayectorias
import numpy as np
import image_control as ic
from optimControl import optimControl
import time
import pandas as pd


####
# region Empty
XEE = []
YEE = []
ZEE = []

XD = []
YD = []
ZD = []

VEL1 = []
VEL2 = []
VEL3 = []
VEL4 = []

ANG1 = []
ANG2 = []
ANG3 = []
ANG4 = []

raw_data = {'theta1': [],
            'theta2': [],
            'theta3': [],
            'theta4': [],
            'pulsos1': [],
            'pulsos2': [],
            'pulsos3': [],
            'pulsos4': [],
            'velocidad1': [],
            'velocidad2': [],
            'velocidad3': [],
            'velocidad4': []}

ERRORX = []
ERRORY = []
ERRORZ = []

# endregion
# endregion
total = time.time()
################################################################################
# region CONNECT TO SERVER
print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
# endregion
################################################################################
# region SAVIORS (COMMENTED)
#file = open('values/THETASNPULSES.csv', 'w')
# endregion
if clientID != -1:
    print('Connected to remote API server')
    ############################################################################
    # region SyncSim
    # start the simulation/enable the synchronous mode on the client:
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    s = sim.simxSynchronous(clientID, True)
    sim.simxSynchronous(clientID, True)
    # endregion
    ############################################################################
    # region PARAMETERS
    t = 0
    dt = 0.005
    T = 70
    To = 0.1  # 0.3
    n = np.transpose(np.arange(0, T, To))
    a = 0.1
    R = 0.04
    K1 = 0.9289 # ;%0.9289;
    K2 = 0.9289
    K3 = 0.9289  # ;%0.5805
    xc = 0
    yc = 0
    zc = 0.08
    b = 0.00025
    p = -0.65
    q = 0.65
    to = 0
    tf = 3
    CVector = np.linspace(to, tf, 100 * (tf - to))

    # endregion
    ############################################################################
    # region HANDLERS
    # let's begin with the object handlers
    joints = vrep_obj.joint_handler(clientID, 'jointR1', 'jointR2', 'jointR3', 'jointR4')
    # print(f'Joint_Handle saved as:\n{joints.jointsDict}')
    objects = vrep_obj.object_handler(clientID, 'Tip')
    # print(f'Object_Handle saved as:\n{objects.objDict}')

    # endregion
    ############################################################################
    # region TRAJECTORY TO FOLLOW
    # trajectory = Trayectorias.cuadrado_trayectory(T)
    # trajectory = Trayectorias.circular_trajectory(To, n, R)

    tic = time.time()
    imagen = ic.Image_Processor('Micelaneos')
    imagen.main_Control()
    imagen.adecuar_al_brazo(0.05, 0.2)
    # imagen.display_Img()
    print(imagen.cCoords)
    print('Tracking Trajectory')
    trajectory = Trayectorias.arreglar_trayectoria(imagen.cCoords, np.ones(len(imagen.cCoords)),
                                                   xc, yc, zc)
    print('Tardó:',time.time() - tic,'s en generar la trayectoria')
    for j in range(70):
        joints.setVelocitiesToJoints(0, 0.1, -0.1, -0.08)
        sim.simxSynchronousTrigger(clientID)

    # endregion
    ############################################################################
    # region Initial Velocity
    tic = time.time()
    joints.setEveryVelocityTo(0)
    # endregion
    ############################################################################
    # region TCONTROL
    XYZEE, XYZD, velo, angle = optimControl(CVector, clientID, joints, objects,
                 trajectory[0][0], trajectory[1][0], trajectory[2][0],
                 To, to, tf, K1, K2, K3)
    # region Save
    for i in range(len(XYZEE[0])):
        XEE.append(XYZEE[0][i])
        YEE.append(XYZEE[1][i])
        ZEE.append(XYZEE[2][i])

        XD.append(XYZD[0][i])
        YD.append(XYZD[1][i])
        ZD.append(XYZD[2][i])

        VEL1.append(velo[0][i])
        VEL2.append(velo[1][i])
        VEL3.append(velo[2][i])
        VEL4.append(velo[3][i])

        ANG1.append(angle[0][i])
        ANG2.append(angle[1][i])
        ANG3.append(angle[2][i])
        ANG4.append(angle[3][i])

        raw_data['theta1'].append(angle[0][i])
        raw_data['theta2'].append(angle[1][i])
        raw_data['theta3'].append(angle[2][i])
        raw_data['theta4'].append(angle[3][i])

        raw_data['pulsos1'].append(joints.getRad('jointR1'))
        raw_data['pulsos2'].append(joints.getRad('jointR2'))
        raw_data['pulsos3'].append(joints.getRad('jointR3'))
        raw_data['pulsos4'].append(joints.getRad('jointR4'))

        raw_data['velocidad1'].append(velo[0][i][0] * 250 / 4.55)
        raw_data['velocidad2'].append(velo[1][i][0] * 250 / 4.55)
        raw_data['velocidad3'].append(velo[2][i][0] * 250 / 4.55)
        raw_data['velocidad4'].append(velo[3][i][0] * 250 / 4.55)
    # endregion
    print('Tip ready')
    print('Tardó:', time.time() - tic,'s en llegar al punto inicial')
    # endregion
    ############################################################################
    # region SEGUIMIENTO DE TRAYECTORIA
    tic = time.time()
    for i in range(len(trajectory[0]) - 1):
        ########################################################################
        # region REFERENCIAS
        xd1 = trajectory[0][i + 1]
        yd1 = trajectory[1][i + 1]
        zd1 = trajectory[2][i + 1]

        xd = trajectory[0][i]
        yd = trajectory[1][i]
        zd = trajectory[2][i]

        sim.simxSynchronousTrigger(clientID)

        xee = objects.getPos('Tip', -1)[0]
        yee = objects.getPos('Tip', -1)[1]
        zee = objects.getPos('Tip', -1)[2]

        XEE.append(xee)
        YEE.append(yee)
        ZEE.append(zee)

        XD.append(xd)
        YD.append(yd)
        ZD.append(zd)
        # endregion
        ########################################################################
        # region CONTROL LAW
        jacobiano = vrep_obj.jacobiano(joints.getRad('jointR1'), joints.getRad('jointR2'),
                                       joints.getRad('jointR3'), joints.getRad('jointR4'))
        control = vrep_obj.control_law(clientID, jacobiano,
                                       xd1, yd1, zd1, xd, yd, zd, xee, yee, zee,
                                       K1, K2, K3, To)

        norma = np.sqrt(np.power(xd1 - xd, 2) + np.power(yd1 - yd, 2) + np.power(zd1 - zd, 2))
        if norma > 0.001:
            print(f'Out of Control with {norma}')
            XYZEE, XYZD, velo, angle  = optimControl(np.linspace(to, tf, 50 * (tf - to)), clientID, joints, objects,
                         xd, yd, zd,
                         To, to, tf, K1, K2, K3)
            ####################################################################
            # region Save
            for m in range(len(XYZEE[0])):
                XEE.append(XYZEE[0][m])
                YEE.append(XYZEE[1][m])
                ZEE.append(XYZEE[2][m])

                XD.append(XYZD[0][m])
                YD.append(XYZD[1][m])
                ZD.append(XYZD[2][m])

                VEL1.append(velo[0][m])
                VEL2.append(velo[1][m])
                VEL3.append(velo[2][m])
                VEL4.append(velo[3][m])

                ANG1.append(angle[0][m])
                ANG2.append(angle[1][m])
                ANG3.append(angle[2][m])
                ANG4.append(angle[3][m])

                raw_data['theta1'].append(angle[0][m])
                raw_data['theta2'].append(angle[1][m])
                raw_data['theta3'].append(angle[2][m])
                raw_data['theta4'].append(angle[3][m])

                raw_data['pulsos1'].append(joints.getRad('jointR1'))
                raw_data['pulsos2'].append(joints.getRad('jointR2'))
                raw_data['pulsos3'].append(joints.getRad('jointR3'))
                raw_data['pulsos4'].append(joints.getRad('jointR4'))

                raw_data['velocidad1'].append(velo[0][m] * 250 / 4.55)
                raw_data['velocidad2'].append(velo[1][m] * 250 / 4.55)
                raw_data['velocidad3'].append(velo[2][m] * 250 / 4.55)
                raw_data['velocidad4'].append(velo[3][m] * 250 / 4.55)

                ERRORX.append(XYZEE[0][m] - XYZD[0][m])
                ERRORY.append(XYZEE[1][m] - XYZD[1][m])
                ERRORZ.append(XYZEE[2][m] - XYZD[2][m])
            # endregion
            ####################################################################
        else:
            if control[1] > 0.3 or control[2] > 0.3 or control[3] > 0.3:
                control[0], control[1], control[2], control[3] = 0.2, 0.2, 0.2, 0.2
            if control[1] < -0.3 or control[2] < -0.3 or control[3] < -0.3:
                control[0], control[1], control[2], control[3] = 0.2, 0.2, -0.2, -0.2

            if i < 1:
                joints.setEveryVelocityTo(0)
            else:
                joints.setVelocitiesToJoints(control[0], control[1], control[2], control[3])  # m * 3
            ####################################################################
            # region save
            XEE.append(xee)
            YEE.append(yee)
            ZEE.append(zee)

            XD.append(xd)
            YD.append(yd)
            ZD.append(zd)

            VEL1.append(control[0])
            VEL2.append(control[1])
            VEL3.append(control[2])
            VEL4.append(control[3])

            ANG1.append(joints.getRad('jointR1'))
            ANG2.append(joints.getRad('jointR2'))
            ANG3.append(joints.getRad('jointR3'))
            ANG4.append(joints.getRad('jointR4'))

            raw_data['theta1'].append(joints.getRad('jointR1'))
            raw_data['theta2'].append(joints.getRad('jointR2'))
            raw_data['theta3'].append(joints.getRad('jointR3'))
            raw_data['theta4'].append(joints.getRad('jointR4'))

            raw_data['pulsos1'].append(joints.getRad('jointR1') * 250 / 4.55)
            raw_data['pulsos2'].append(joints.getRad('jointR2') * 250 / 4.55)
            raw_data['pulsos3'].append(joints.getRad('jointR3') * 250 / 4.55)
            raw_data['pulsos4'].append(joints.getRad('jointR4') * 250 / 4.55)

            raw_data['velocidad1'].append(control[0])
            raw_data['velocidad2'].append(control[1])
            raw_data['velocidad3'].append(control[2])
            raw_data['velocidad4'].append(control[3])

            ERRORX.append(xee - xd)
            ERRORY.append(yee - yd)
            ERRORZ.append(zee - zd)
            # endregion
            ####################################################################
        pulsos = [joints.getRad('jointR1') * 250 / 4.55,
                  joints.getRad('jointR2') * 250 / 4.55,
                  joints.getRad('jointR3') * 250 / 4.55,
                  joints.getRad('jointR4') * 250 / 4.55]
        # print('CONTROL VELOCITIES:', control)
        # print('JACOBIAN:', jacobiano)
        print('Step: {}\tStatus:'
              '\n\tCurrent tip location: {} {} {}'
              '\n\tMoving to: {} {} {}'
              '\n\tVelocity given: \n\t\t{}'
              '\n\tPulsos: \n\t\t{}'.format(i, xee, yee, zee, xd, yd, zd, control, pulsos))
        # endregion
        ########################################################################
    print('Tardó:', time.time() - tic,'s en crear la figura')
    # endregion
    ############################################################################
    # region SAVE TO EXCEL
    df = pd.DataFrame(raw_data, columns=['theta1', 'theta2', 'theta3', 'theta4',
                                         'pulsos1', 'pulsos2', 'pulsos3', 'pulsos4',
                                         'velocidad1', 'velocidad2', 'velocidad3', 'velocidad4'])

    df.to_csv('squareNotDrawedFigure.csv', index=False)

    print('PERFIL CUADRADO:',vrep_obj.error(ERRORX, To), vrep_obj.error(ERRORY, To), vrep_obj.error(ERRORZ, To))
    # endregion
    ############################################################################
    # region PLOTS
    # tip y deseado

    def error(*args):
        e = []
        for j in range(len(args[0])):
            e.append(abs(args[0][j] - args[1][j]))
        return e

    # posiciones
    fig, axs = plt.subplots(2)
    t = np.linspace(0, len(XEE), len(XEE))
    fig.suptitle('tip, deseado y error')
    axs[0].plot(XEE, YEE, color='blue', linewidth='0.5', label='tip')
    #axs[0].plot(XD, YD, color='red', linewidth='0.4', label='expected')
    axs[1].plot(t, error(XEE, XD), color='blue',
                linewidth='0.5',
                label='error en X')
    axs[1].plot(t, error(YEE, YD), color='red',
                linewidth='0.4',
                label='error en Y')
    # Velocidades y angulos
    fig2, axs2 = plt.subplots(2)
    fig2.suptitle('Velocidades y angulos respecto del tiempo')
    t = np.linspace(1, len(velo[0]), len(velo[0]))
    axs2[0].plot(t, velo[0],linestyle=(5, (10, 3)), color='b', label='velocidad R1')
    axs2[0].plot(t, velo[1],linestyle=(0, (5, 1)), color='r', label='velocidad R2')
    axs2[0].plot(t, velo[2],linestyle=(0, (1, 1)), color='k', label='velocidad R3')
    axs2[0].plot(t, velo[3],linestyle='solid', color='m', label='velocidad R4')
    t = np.linspace(1, len(angle[0]), len(angle[0]))
    axs2[1].plot(t, angle[0], linestyle=(5, (10, 3)), color='b', label='angulo R1')
    axs2[1].plot(t, angle[1], linestyle=(0, (5, 1)), color='r', label='angulo R2')
    axs2[1].plot(t, angle[2], linestyle=(0, (1, 1)), color='k', label='angulo R3')
    axs2[1].plot(t, angle[3], linestyle='solid', color='m', label='angulo R4')
    plt.show()
    # error tip respecto del tiempo


    # endregion
    ############################################################################
    # region STOP SIM
    # stop the simulation:
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
    # endregion
    ############################################################################
    print('El proceso tardó:', time.time()-total)
else:
    print('Failed connecting to remote API server')



print('Program ended')
