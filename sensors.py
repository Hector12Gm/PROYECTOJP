from calendar import c
import pwd
import numpy as np 

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