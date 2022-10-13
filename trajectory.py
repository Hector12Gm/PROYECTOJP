
from hashlib import new
from queue import Empty
from matplotlib.pyplot import sci
import numpy as  np
import sys
import scipy.interpolate as spi
import math as m
import sim
import matplotlib.pyplot as plt
import time


xt = []
yt = []

Kv = 1
Kh = 2.5


t=time.time

hd = 0
r = 0.5*0.195
L = 0.311
errp = 10

#xarr = np.array([0,0.5,1,2.5,3, 3, 3,3,2.5,0.5,0, 0,0, 0,0])
#yarr = np.array([0, 0,0, 0,0,0.5,2.5,3, 3, 3,3,2.5,1,0.5,0])

xarr = np.random.randint(0,5,10)
yarr = np.random.randint(0,5,10)

plt.scatter(xarr,yarr)

def create_neighbor(x,r,n):

    increment = r / n

    x_new = []
    
    i = n 
    while i >= 0:
        x_new.append( x - increment * i)
        i -= 1

    i = 1
    while i < n +1:
        x_new.append(x + increment * i)
        i += 1
    return np.array(x_new)


def new_points(X):
    x_new = []

    for x in X:
        inter_x = create_neighbor(x,5,5)
        for i_x in inter_x:
            x_new.append(i_x)

    return np.array(x_new)

#xarr = new_points(xarr)
#yarr = new_points(yarr)


def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

def interp(tiempo,x,y):
    ttime =  70
    tarr = np.linspace(0, ttime, x.shape[0])
    #TIempo  
    xc = spi.splrep(tarr, x, s=0)
    yc = spi.splrep(tarr, y, s=0)

  
    xnew = spi.splev(tiempo, xc, der=0)
    ynew = spi.splev(tiempo, yc, der=0)

    return {'x':xnew,'y':ynew}

def Trajectory(client_id, robot,ts,t):

    
    tau = ts  -  t
    ret, carpos = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_blocking)
    ret, carrot = sim.simxGetObjectOrientation(client_id, robot, -1, sim.simx_opmode_blocking)

    data = interp(tau,xarr,yarr)
    xd = data['x']
    yd = data['y']

    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angdiff(carrot[2], angd)
    print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

    v = Kv*errp
    omega = Kh*errh

    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)

    xt.append(carpos[0])
    yt.append(carpos[1])
    
        #time.sleep(0.1)
    return(ul,ur,xt,yt)
    
