import numpy as np 
import time 
import sys
import sim 
import matplotlib.pyplot as plt
import trajectory as tj
import math as m

sim.simxFinish(-1) #Finaliza la ejecucion 
client_id = sim.simxStart('127.0.0.1',-1,True,True,5000,5)

if client_id != -1:
    print('Connection done')
else:
    print('Not connection')
    sys.exit('Error, cant not connect to the simulator')

err, motor_l = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
err, motor_r = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(client_id, 'Robot', sim.simx_opmode_blocking)

Kv = 1
Kh = 2.5

# xd and yd are the coordinates of the desired setpoint
xd = 3
yd = 3

xt = []
yt = []

hd = 0
r = 0.5*0.195
L = 0.311
errp = 10

xarr = np.array([0,0.5,1,2.5,3, 3, 3,3,2.5,0.5,0, 0,0, 0,0])
yarr = np.array([0, 0,0, 0,0,0.5,2.5,3, 3, 3,3,2.5,1,0.5,0])

plt.scatter(xarr,yarr)
t = time.time()

while (time.time()-t)  < 71.5:
   
    ts = time.time()

    tau = ts  -  t
    ret, carpos = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_blocking)
    ret, carrot = sim.simxGetObjectOrientation(client_id, robot, -1, sim.simx_opmode_blocking)

    data = tj.interp(tau,xarr,yarr)
    xd = data['x']
    yd = data['y']

    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = tj.angdiff(carrot[2], angd)
    print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

    v = Kv*errp
    omega = Kh*errh

    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_l, ul, sim.simx_opmode_streaming)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_r, ur, sim.simx_opmode_streaming)
        #time.sleep(0.1)
    xt.append(carpos[0])
    yt.append(carpos[1])
        
plt.scatter(xt,yt)


plt.show()
for i in range(10):
    errf = sim.simxSetJointTargetVelocity(client_id, motor_l, 0, sim.simx_opmode_streaming)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_r, 0, sim.simx_opmode_streaming)
    #time.sleep(0.1)
    
sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)
