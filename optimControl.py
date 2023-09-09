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

def optimControl(CVector, clientID, joints, objects, xeed, yeed, zeed, To, to, tf, K1, K2, K3):
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

    for i in range(5):
        joints.setEveryVelocityTo(0)
        sim.simxSynchronousTrigger(clientID)

    tip_pos = objects.getPos('Tip', -1)

    print('Making a beginning rute to follow')
    print(xeed, yeed, zeed)
    XXd, Xvd = vrep_obj.rute_control(tip_pos[0], 0, xeed, To, to, tf)
    YYd, Yvd = vrep_obj.rute_control(tip_pos[1], 0, yeed, To, to, tf)
    ZZd, Zvd = vrep_obj.rute_control(tip_pos[2], 0, zeed, To, to, tf)

    for k in range(len(CVector)):

        if k == 1:
            print('Following the rute')
        """if rad_f_joint[3] >= -np.pi / 4:
            rad_f_joint[3] = -np.pi / 4"""
        ###########
        # region REFERENCIAS
        xd1 = XXd[k]
        yd1 = YYd[k]
        zd1 = ZZd[k]

        xd = XXd[k - 1]
        yd = YYd[k - 1]
        zd = ZZd[k - 1]

        sim.simxSynchronousTrigger(clientID)

        xee = objects.getPos('Tip', -1)[0]
        yee = objects.getPos('Tip', -1)[1]
        zee = objects.getPos('Tip', -1)[2]

        if k < 1:
            xd = xee
            yd = yee
            zd = zee

        XEE.append(xee)
        YEE.append(yee)
        ZEE.append(zee)

        XD.append(xd)
        YD.append(yd)
        ZD.append(zd)

        # endregion
        ###########
        jacobiano = vrep_obj.jacobiano(joints.getRad('jointR1'), joints.getRad('jointR2'),
                                       joints.getRad('jointR3'), joints.getRad('jointR4'))
        # print("DESEADA, DESEADA-1, EFECTOR:", xd1, yd1, zd1, xd, yd, zd, xee, yee, zee)
        control = vrep_obj.control_law(clientID, jacobiano,
                                       xd1, yd1, zd1, xd, yd, zd, xee, yee, zee,
                                       K1, K2, K3, To)

        if control[1] > 0.3 or control[2] > 0.3 or control[3] > 0.3:
            control[0], control[1], control[2], control[3] = 0.2, 0.2, 0.2, 0.2
        if control[1] < -0.3 or control[2] < -0.3 or control[3] < -0.3:
            control[0], control[1], control[2], control[3] = 0.2, 0.2, -0.2, -0.2

        joints.setVelocitiesToJoints(control[0], control[1], control[2], control[3])

        VEL1.append(control[0])
        VEL2.append(control[1])
        VEL3.append(control[2])
        VEL4.append(control[3])

        ANG1.append(joints.getRad('jointR1'))
        ANG2.append(joints.getRad('jointR2'))
        ANG3.append(joints.getRad('jointR3'))
        ANG4.append(joints.getRad('jointR4'))


    return [XEE, YEE, ZEE], [XD, YD, ZD], [VEL1, VEL2, VEL3, VEL4], [ANG1, ANG2, ANG3, ANG4]