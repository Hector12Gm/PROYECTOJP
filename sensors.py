from calendar import c
import numpy as np 
import sim



def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1-2*(x**2+y**2)
    return R



def ref_world(srot,spos,cpos,crot,uread,point):
    
    
    R = q2R(srot[0], srot[1], srot[2], srot[3])
    spos = np.array(spos).reshape((3,1))
    point = [0, 0, uread]
    ps = np.array(point).reshape((3,1))
    pc = R.dot(ps) + spos
    pw = np.dot( q2R(crot[0],crot[1],crot[2]),pc) + cpos
    return pw

def Braitenberg (client_id, robot):
        # Assigning handles to the ultrasonic sensors
    usensor = []
    for i in range(1,17):
        err, s = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor'+str(i), sim.simx_opmode_blocking)
        usensor.append(s)

    # Sensor initialization
    for i in range(16):
        err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(client_id, usensor[i], sim.simx_opmode_streaming)

    ret, carpos = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_streaming)
    ret, carrot = sim.simxGetObjectOrientation(client_id, robot, -1, sim.simx_opmode_streaming)

    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = np.zeros(16)
    braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    for i in range (16):
        err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(client_id, usensor[i], sim.simx_opmode_streaming)
        dist = np.linalg.norm(point) 
        if state and dist<noDetectionDist:
            if dist<maxDetectionDist:
                dist=maxDetectionDist
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else:
            detect[i]=0
    
    vLeft=2
    vRight=2
    
    for i in range (16):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]

    
        
    return vLeft, vRight








