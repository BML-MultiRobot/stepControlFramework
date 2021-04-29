import vrep
import numpy as np
import sys
import cv2
import time
import matplotlib.pyplot as plt
import math

LSignalName = "CycleLeft"
RSignalName = "CycleRight"
BaseFreq = -5

factor = 1
factor1 = 1

class Pursuit3:
    def __init__(self, client, leftName, rightname, vis, followc1, followc2, body):
        #self.cam = vis
        self.go = True
        self.clientID = client
        self.LSignalName = leftName
        self.RSignalName = rightname
        self.visionname = vis

        self.LCycleFreq = BaseFreq
        self.RCycleFreq = BaseFreq

        self.orient = None
        self.kp = 1
        self.kd = -0.3
        self.ki = 0.2
        self.x = None
        self.y = None
        self.prevE = 0
        self.totalE = 0

        self.pointx = 0
        self.pointy = 0
        self.pos = None

        self.lower_blue = followc1
        self.upper_blue = followc2

        # for test :: to get the handle
        e, self.cam = vrep.simxGetObjectHandle(clientID, self.visionname, vrep.simx_opmode_blocking)
        e, self.cube = vrep.simxGetObjectHandle(clientID, body, vrep.simx_opmode_blocking)
        e, self.pos = vrep.simxGetObjectPosition(clientID, self.cube, -1, vrep.simx_opmode_streaming)
        e, self.orient = vrep.simxGetObjectOrientation(clientID, self.cube, -1, vrep.simx_opmode_streaming)

        self.pointx, self.pointy = self.pos[:2]

    def clearSignal(self):
        """
        clear the signal at the very begining
        """
        vrep.simxClearFloatSignal(self.clientID, self.LSignalName, vrep.simx_opmode_oneshot)
        vrep.simxClearFloatSignal(self.clientID, self.RSignalName, vrep.simx_opmode_oneshot)

    def preparation(self):
        """
        implement localization and getPath
        """

        e, self.pos = vrep.simxGetObjectPosition(clientID, self.cube, -1, vrep.simx_opmode_buffer )
        e, self.orient = vrep.simxGetObjectOrientation(clientID, self.cube, -1, vrep.simx_opmode_buffer )

    def publish(self):
        """
        send msg to vrep
        msg : the frequency of leg
        """
        vrep.simxSetFloatSignal(self.clientID, self.LSignalName, self.LCycleFreq, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(self.clientID, self.RSignalName, self.RCycleFreq, vrep.simx_opmode_oneshot)


    def controller(self):
        if self.go:
            self.LCycleFreq = factor * BaseFreq
            self.RCycleFreq = factor1 * BaseFreq
            self.publish()

    def stepLeft(self, freq):
        if self.go:

            #self.publish()
            vrep.simxSetFloatSignal(self.clientID, "basefreq", freq, vrep.simx_opmode_oneshot)
            vrep.simxSetIntegerSignal(self.clientID, "setting", 1, vrep.simx_opmode_oneshot)
            print("set 1")
            while vrep.simxGetIntegerSignal(self.clientID, "setting", vrep.simx_opmode_oneshot) == 1:
                print("waiting")
                continue

    def stepLeftPercent(self, freq, percent):
        if self.go:

            #self.publish()
            vrep.simxSetFloatSignal(self.clientID, "percent", percent, vrep.simx_opmode_oneshot)
            vrep.simxSetFloatSignal(self.clientID, "basefreq", freq, vrep.simx_opmode_oneshot)
            vrep.simxSetIntegerSignal(self.clientID, "setting", 3, vrep.simx_opmode_oneshot)
            print("set 1")
            while vrep.simxGetIntegerSignal(self.clientID, "setting", vrep.simx_opmode_oneshot) == 3:
                print("waiting")
                continue

    def stepRight(self, freq):
        if self.go:

            #self.publish()
            vrep.simxSetFloatSignal(self.clientID, "basefreq", freq, vrep.simx_opmode_oneshot)
            vrep.simxSetIntegerSignal(self.clientID, "setting", 2, vrep.simx_opmode_oneshot)
            print("set 1")
            while vrep.simxGetIntegerSignal(self.clientID, "setting", vrep.simx_opmode_oneshot) == 2:
                print("waiting")
                continue

    def stepRightPercent(self, freq, percent):
        if self.go:

            #self.publish()
            vrep.simxSetFloatSignal(self.clientID, "percent", percent, vrep.simx_opmode_oneshot)
            vrep.simxSetFloatSignal(self.clientID, "basefreq", freq, vrep.simx_opmode_oneshot)
            vrep.simxSetIntegerSignal(self.clientID, "setting", 4, vrep.simx_opmode_oneshot)
            print("set 1")
            while vrep.simxGetIntegerSignal(self.clientID, "setting", vrep.simx_opmode_oneshot) == 4:
                print("waiting")
                continue

    def stop(self):
        self.LCycleFreq = 0
        self.RCycleFreq = 0
        self.publish()

# vrep stuff
vrep.simxFinish(-1)  # clean up the previous stuff
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection unsuccessful')
    sys.exit('Error: Could not connect to API server')


pursuit = Pursuit3(clientID, LSignalName, RSignalName, 'visi1', np.array([150,120,70]), np.array([200, 255, 255]), "body#1")
error, res, i = vrep.simxGetVisionSensorImage(clientID, pursuit.cam, 0, vrep.simx_opmode_streaming)

time.sleep(2)
counter = 0

pursuit.clearSignal()
code, t = vrep.simxGetFloatSignal(clientID, "time", vrep.simx_opmode_streaming)
print(t)
time.sleep(3)
code, t = vrep.simxGetFloatSignal(clientID, "time", vrep.simx_opmode_streaming)
print(t)

f = []
degPsec = []

e, reference = vrep.simxGetObjectHandle(clientID, "reference", vrep.simx_opmode_blocking)

vrep.simxSetObjectOrientation(clientID, reference, -1, pursuit.orient, vrep.simx_opmode_oneshot)

#command = "L50--"
time.sleep(2)

results = [['LLLRR--', 0.018604116095229984, -0.011968761682510376, 0.10418207561087911], ['LLRLLR--', 0.03806414306163788, -0.010302779078483582, 0.19507652712782148], ['LLLR--', 0.009966114163398742, -0.003252634406089783, 0.04280867483175825], ['L50L50L50R50R50--', 0.08388174176216126, 0.026041057705879212, 0.2945983968913879], ['L20L20L20R20R20--', 0.04542134702205658, 0.010160261392593383, 0.20900284571252997], ['L20L20L20R20R20--', 0.032924997806549075, 0.009898483753204346, 0.12037271795013708]]
columns = ('Command', 'X', 'Y', 'Turn')
print(results)

fig, ax = plt.subplots()
ax.set_axis_off()
table = ax.table(
    cellText=results,
    colLabels=columns,
    cellLoc='center',
    loc='upper left',
    fontsize = 20)

plt.show()

commands = ["LLLRR--", "LLRLLR--", "LLLR--", "L50L50L50R50R50--", "L20L20L20R20R20--", "L20L20L20R20R20--"]

#commands = ["LLLRR--", "LLRLLR--", "LLLR--"]


results = []
for command in commands:
    resx = []
    resy = []
    resdeg = []
    for _ in range(5):

        pursuit.stop()
        pursuit.preparation()
        vrep.simxSetObjectOrientation(clientID, reference, -1, pursuit.orient, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectPosition(clientID, reference, -1, [pursuit.pos[0], pursuit.pos[1], -5], vrep.simx_opmode_oneshot)
        time.sleep(1)
        counter = 0
        code, tstart = vrep.simxGetFloatSignal(clientID, "time", vrep.simx_opmode_streaming)
        #degstart = pursuit.orient[2]
        e, degstart = vrep.simxGetObjectOrientation(clientID, pursuit.cube, reference, vrep.simx_opmode_streaming)
        e, posstart = vrep.simxGetObjectPosition(clientID, pursuit.cube, reference, vrep.simx_opmode_streaming)
        degstart = degstart[2]
        print("test started")


        for i in range(len(command)):
            if command[i].isnumeric():
                continue
            elif command[i] == '-':
                break
            elif command[i + 2].isnumeric() and command[i + 1].isnumeric():
                percent = int(command[i + 1:i + 2] + command[i + 2:i + 3])
                if command[i] == 'L':
                    print("leftpercent")
                    pursuit.stepLeftPercent(4, percent)
                    time.sleep(2)
                else:
                    print("rightpercent")
                    pursuit.stepRightPercent(4, percent)
                    time.sleep(2)
            else:
                if command[i] == 'L':
                    print("left")
                    pursuit.stepLeft(4)
                    time.sleep(2)
                else:
                    print("right")
                    pursuit.stepRight(4)
                    time.sleep(2)

        #degend = pursuit.orient[2]


        e, degend = vrep.simxGetObjectOrientation(clientID, pursuit.cube, reference, vrep.simx_opmode_streaming)
        e, posend = vrep.simxGetObjectPosition(clientID, pursuit.cube, reference, vrep.simx_opmode_streaming)
        degend = degend[2]
        d = degend - degstart
        x =  posstart[0] - posend[0]
        y = posstart[1] - posend[1]
        #code, tend = vrep.simxGetFloatSignal(clientID, "time", vrep.simx_opmode_streaming)
        #degPsec.append(d/(tend - tstart))
        resx.append(x)
        resy.append(y)
        resdeg.append(d)


    aresx = sum(resx)/5
    aresy = sum(resy) / 5
    aresdeg = sum(resdeg) / 5

    results.append([command, aresx, aresy, aresdeg])


columns = ('Command', 'X', 'Y', 'Turn')
print(results)

fig, ax = plt.subplots()
ax.set_axis_off()
table = ax.table(
    cellText=results,
    colLabels=columns,
    cellLoc='center',
    loc='upper left',
    font = 20)

plt.show()
