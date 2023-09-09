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

import numpy as np

########################################################################
# region SIMULATION CLASSES FOR GETTER AND SETTER IN SIM
class joint_handler:

    def __init__(self, clientID, *Joints):
        self.clientID = clientID
        self.errorCodes = {}
        self.jointsDict = {}
        for joint in Joints:
            # Getting object handles
            self.errorCodes[joint], self.jointsDict[joint] = sim.simxGetObjectHandle(self.clientID, joint,
                                                                                     sim.simx_opmode_oneshot_wait)

    def getRad(self, joint):
        # getting the current angle (blocking)
        return sim.simxGetJointPosition(self.clientID, self.jointsDict[joint],
                                        sim.simx_opmode_blocking)[1]

    def setVelocity(self, joint, velocity):
        # setting the velocity
        return sim.simxSetJointTargetVelocity(self.clientID, self.jointsDict[joint], velocity, sim.simx_opmode_oneshot)

    def setEveryVelocityTo(self, velocity):
        for joint in self.jointsDict:
            sim.simxSetJointTargetVelocity(self.clientID, self.jointsDict[joint], velocity, sim.simx_opmode_oneshot)

    def getJointHandle(self, joint):
        return self.jointsDict[joint]

    def setJointHandle(self, joint):
        self.jointsDict[joint] = sim.simxGetObjectHandle(self.clientID, joint, sim.simx_opmode_oneshot_wait)

    def getEveryRad(self):
        rad = []
        for joint in self.jointsDict:
            rad.append(sim.simxGetJointPosition(self.clientID, self.jointsDict[joint],
                                                sim.simx_opmode_blocking)[1])
        return rad

    def setVelocitiesToJoints(self, *args):
        for i, joint in enumerate(self.jointsDict):
            sim.simxSetJointTargetVelocity(self.clientID, self.jointsDict[joint], args[i], sim.simx_opmode_oneshot)


class object_handler:

    def __init__(self, clientID, *Object):
        self.clientID = clientID
        self.errorCodes = {}
        self.objDict = {}
        for obj in Object:
            self.errorCodes[obj], self.objDict[obj] = sim.simxGetObjectHandle(self.clientID, obj,
                                                                              sim.simx_opmode_oneshot_wait)

    def getPos(self, obj, refer_obj):
        # getting pos of any obj
        if refer_obj == -1:
            return sim.simxGetObjectPosition(self.clientID, self.objDict[obj], -1,
                                             sim.simx_opmode_blocking)[1]
        return sim.simxGetObjectPosition(self.clientID, self.objDict[obj], self.objDict[refer_obj],
                                         sim.simx_opmode_blocking)[1]

    def setPos(self, obj, refer_obj, coords: list):
        # setting pos
        return sim.simxSetObjectPosition(self.clientID, self.objDict[obj], self.objDict[refer_obj], coords,
                                         sim.simx_opmode_oneshot_wait)

    def getObjectHandle(self, obj):
        return self.objDict[obj]

    def setObjectHandle(self, obj):
        self.objDict[obj] = sim.simxGetObjectHandle(self.clientID, obj, sim.simx_opmode_oneshot_wait)


# endregion
########################################################################
# region FUNCTIONS FOR CONTROL
def multi_dot(*args):
    """
    With numpy we do dot operation to each argument given sequentially
    :param args: Every matrix, float or int you want to dot sequentially
    :return: dot result
    """
    initial = args[0]
    for i in range(1, len(args)):
        m = np.dot(initial, args[i])
        initial = m
    return initial


def multi_multiply(*args):
    """
    With numpy we do dot operation to each argument given sequentially
    :param args: Every matrix, float or int you want to dot sequentially
    :return: dot result
    """
    initial = args[0]
    for i in range(1, len(args)):
        m = np.multiply(initial, args[i])
        initial = m
    return initial


def jacobiano(rot1, rot2, rot3, rot4):
    """
    l1 = 0.33
    l2 = 0.96
    l3 = 0.95
    l4 = 0.53
    """

    l1 = 0.1140
    l2 = 0.1120
    l3 = 0.0620

    J11 = -np.sin(rot1) * (l2 * np.cos(rot2 + rot3) + l1 * np.cos(rot2) + l3 * np.cos(rot2 + rot3 + rot4))
    J12 = -np.cos(rot1) * (l2 * np.sin(rot2 + rot3) + l1 * np.sin(rot2) + l3 * np.sin(rot2 + rot3 + rot4))
    J13 = -np.cos(rot1) * (l2 * np.sin(rot2 + rot3) + l3 * np.sin(rot2 + rot3 + rot4))
    J14 = -l3 * np.sin(rot2 + rot3 + rot4) * np.cos(rot1)

    J21 = np.cos(rot1) * (l2 * np.cos(rot2 + rot3) + l1 * np.cos(rot2) + l3 * np.cos(rot2 + rot3 + rot4))
    J22 = -np.sin(rot1) * (l2 * np.sin(rot2 + rot3) + l1 * np.sin(rot2) + l3 * np.sin(rot2 + rot3 + rot4))
    J23 = -np.sin(rot1) * (l2 * np.sin(rot2 + rot3) + l3 * np.sin(rot2 + rot3 + rot4))
    J24 = -l3 * np.sin(rot2 + rot3 + rot4) * np.sin(rot1)

    J31 = 0
    J32 = l2 * np.cos(rot2 + rot3) + l1 * np.cos(rot2) + l3 * np.cos(rot2 + rot3 + rot4)
    J33 = l2 * np.cos(rot2 + rot3) + l3 * np.cos(rot2 + rot3 + rot4)
    J34 = l3 * np.cos(rot2 + rot3 + rot4)

    return np.array([[J11, J12, J13, J14],
                     [J21, J22, J23, J24],
                     [J31, J32, J33, J34]])


def control_law(clientID, jacobian, xd1, yd1, zd1, xd, yd, zd, xee, yee, zee, kx, ky, kz, To):
    """
    Used to control the movement of our robot according to any trajectory given
    :param jacobian: Our Jacobian
    :param xd1: Next wished coord x in time
    :param yd1: Next wished coord y in time
    :param zd1: Next wished coord z in time
    :param xd: Actual wished coord x
    :param yd: Actual wished coord y
    :param zd: Actual wished coord z
    :param xee: Actual effector coord x
    :param yee: Actual effector coord y
    :param zee: Actual effector coord z
    :param kx: Constant given to solve coord x mistake
    :param ky: Constant given to solve coord y mistake
    :param kz: Constant given to solve coord z mistake
    :param To: Our interval range
    :return: Velocities for our joints
    """

    """print(
        "Entrada Jacobiano:{}\nEntradaDeseadas+1\n{},{},{}\nEntradaDeseadas\n{},{},{}\nDelEfector\n{},{},{}\nConstantes\n{},{},{}\nTo\n{}".format(
            jacobian, xd1, yd1, zd1, xd, yd, zd, xee, yee, zee, kx, ky, kz, To))"""
    jjt = np.dot(jacobian, np.transpose(jacobian))

    try:
        jjt_1 = np.linalg.inv(jjt)
    except:
        jjt_1 = np.linalg.inv(jjt + np.array([[1 * 10 ** (-10), 0, 0],
                                              [0, 1 * 10 ** (-10), 0],
                                              [0, 0, 1 * 10 ** (-10)], ]))

    jp = np.dot(np.transpose(jacobian), jjt_1)  # pseudoinversa

    ex = np.tanh(xd - xee)
    ey = np.tanh(yd - yee)
    ez = np.tanh(zd - zee)

    jaConstant = np.divide(jp, To)  # pseudoinversa / To

    cMatrix = np.array([[xd1 - kx * ex - xee],
                        [yd1 - ky * ey - yee],
                        [zd1 - kz * ez - zee]])

    return np.dot(jaConstant, cMatrix)

def rute_control(tip_pos, tip_vel, tipd_pos, tipw_vel, to, tf):
    
    t = np.linspace(to, tf, 100 * (tf - to))

    c = np.ones(len(t))

    M = np.array([[1, to, to ** 2, to ** 3, to ** 4, to ** 5, to ** 6],
                  [0, 1, 2 * to, 3 * to ** 2, 4 * to ** 3, 5 * to ** 4, 6 * to ** 5],
                  [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5, tf ** 6],
                  [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4, 6 * tf ** 5]])

    b = np.array([[tip_pos],
                  [tip_vel],
                  [tipd_pos],
                  [tipw_vel]], dtype=object)

    a = multi_dot(np.transpose(M), np.linalg.inv(np.dot(M, np.transpose(M))), b)

    #######
    # region TVARS
    t2 = multi_multiply(t, t)
    t3 = multi_multiply(t, t, t)
    t4 = multi_multiply(t, t, t, t)
    t5 = multi_multiply(t, t, t, t, t)
    t6 = multi_multiply(t, t, t, t, t, t)
    # endregion
    #######

    qd = a[0] * c + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5 + a[6] * t6
    vd = a[1] * c + 2 * a[2] * t + 3 * a[3] * t2 + 4 * a[4] * t3 + 5 * a[5] * t4 + 6 * a[6] * t5
    return [qd, vd]
# endregion
########################################################################
# region AUXILIAR
def error(e, T):
    r = 0
    M = len(e)
    for k in range(0, M):
        r += abs(((e[k] + e[k-1])/2) * T)
    return r

# endregion
########################################################################