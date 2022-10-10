
from matplotlib.pyplot import sci
import numpy as  np
import scipy.interpolate as spi
import math as m



def interp(tiempo,x,y):
    ttime =  70
    tarr = np.linspace(0, ttime, x.shape[0])
    #TIempo  
    xc = spi.splrep(tarr, x, s=0)
    yc = spi.splrep(tarr, y, s=0)

  
    xnew = spi.splev(tiempo, xc, der=0)
    ynew = spi.splev(tiempo, yc, der=0)

    return {'x':xnew,'y':ynew}

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)
