from queue import Empty
import numpy as np 
import time 
import sys
import sensors as sens
import sim 
import matplotlib.pyplot as plt
import trajectory as traject
import math as m


sim.simxFinish(-1) #Finaliza la ejecucion 
client_id = sim.simxStart('127.0.0.1',-1,True,True,5000,5)

if client_id != -1:
    print('Connection done')
else:
    print('Not connection')
    sys.exit('Error, cant not connect to the simulator')

err, motor_l = sim.simxGetObjectHandle(client_id, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
err, motor_r = sim.simxGetObjectHandle(client_id, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
err, robot = sim.simxGetObjectHandle(client_id, '/PioneerP3DX', sim.simx_opmode_blocking)


ts = time.time()
t = time.time()


while (time.time()-t)  < 71.5:
   
    ts = time.time()

    # Algoritmo de Braitenberg
    #for i in range (16):
    #    vLeft, vRight = sens.Braitenberg(client_id, robot)
    #    errf = sim.simxSetJointTargetVelocity(client_id, motor_l, vLeft, sim.simx_opmode_streaming)
    #    errf = sim.simxSetJointTargetVelocity(client_id, motor_r, vRight, sim.simx_opmode_streaming)

    # Seguimiento de trayectoria 
    vLeft, vRight, xt,yt = traject.Trajectory(client_id, robot,ts,t)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_l, vLeft, sim.simx_opmode_streaming)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_r, vRight, sim.simx_opmode_streaming)
    
plt.scatter(xt,yt)
plt.show()
        

for i in range(10):
    errf = sim.simxSetJointTargetVelocity(client_id, motor_l, 0, sim.simx_opmode_streaming)
    errf = sim.simxSetJointTargetVelocity(client_id, motor_r, 0, sim.simx_opmode_streaming)
    #time.sleep(0.1)"""
    
sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)
