
import sim     
import time
import math
import sys
import matplotlib.pyplot as mpl   
import math as m
import numpy as np
import scipy.interpolate as spi
import trajectory as Traject

Kv = 0.2 # 1    0.3
Kh = 0.3 # 2.5  0.5

#Tiempo
END = 600

""" Seleccion de trayectoria """
xarr, yarr, xorg, yorg = Traject.Random()     # T = 400
#xarr, yarr = Traject.square()                # T = 80
#xarr, yarr = Traject.SQUARE()                # T = 350
#xarr, yarr = Traject.Diagonal()              # T = 50


# Imprimimos la trayectoria a seguir
Traject.Grafica(xarr, yarr, xorg, yorg)

class Robot():

    def __init__(self):

        # Asignamos los handles a los motores
        err, self.motor_l = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
        err, self.motor_r = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
        err, self.robot = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)

        # Asignamos los handles a los sensores
        self.usensor = []
        for i in range(16):
            err, s = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), sim.simx_opmode_blocking)
            self.usensor.append(s)

        # Inicializamos los sensores
        for i in range(16):
           err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_streaming)

    def getDistanceReading(self, i):
        # Obtenemos la lectura del sensor
        err, State, Point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)

        if State == 1:
            # retornamos la magnitud del punto detectado
            return math.sqrt(sum(i**2 for i in Point))
        else:
            # Devuelve otro valor que sabemos que no puede ser verdadero
            return 199999

    def stop(self):
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, 0, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, 0, sim.simx_opmode_blocking)

    def angdiff(self, t1, t2):
        # The angle magnitude comes from the dot product of two vectors
        angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
        # The direction of rotation comes from the sign of the cross product of two vectors
        angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
        return m.copysign(angmag, angdir)

    def interp(self, tiempo):


        """ Interpolador Pchip """"" 

        xnew = pcix(tiempo)
        ynew = pciy(tiempo) 
        

        """" Interpolador spline """ """""

        xc = spi.splrep(tarr, xarr, s=0)
        yc = spi.splrep(tarr, yarr, s=0)

        
        xnew = spi.splev(tiempo, xc, der=0)
        ynew = spi.splev(tiempo, yc, der=0)

        """""

        return {'x':xnew,'y':ynew}

    def Trajectory(self):

        ts = time.time()

        ret, carpos = sim.simxGetObjectPosition(clientID, self.robot, -1, sim.simx_opmode_blocking)
        ret, carrot = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)

        tau = ts - t

        data = self.interp(tau)
        xd = data['x']
        yd = data['y']

        errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
        angd = m.atan2(yd-carpos[1], xd-carpos[0])
        errh = self.angdiff(carrot[2], angd)

        v = Kv*errp
        omega = Kh*errh

        ul = v/r - L*omega/(2*r)
        ur = v/r + L*omega/(2*r)

        xt.append(carpos[0])
        yt.append(carpos[1])


        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, ul, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, ur, sim.simx_opmode_blocking)
        
    def Braitenberg (self):
        for i in range (16):
            err, state, point, detectedObj, detectedSurfNormVec = sim.simxReadProximitySensor(clientID, self.usensor[i], sim.simx_opmode_buffer)
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

        err = sim.simxSetJointTargetVelocity(clientID, self.motor_l, vLeft, sim.simx_opmode_blocking)
        err = sim.simxSetJointTargetVelocity(clientID, self.motor_r, vRight, sim.simx_opmode_blocking)

print ('Programa Iniciado')
sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',-1,True,True,5000,5)  # Conexión a CoppeliaSim

if clientID!=-1:
    print ('Conectados al remote API')
    
    robot = Robot() 

    r = 0.5*0.195
    L = 0.311

    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = np.zeros(16)
    braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    xt = []
    yt = []

    ttime =  END

    tarr = np.linspace(0, ttime, xarr.shape[0])
    pcix = spi.PchipInterpolator(tarr, xarr) 
    pciy = spi.PchipInterpolator(tarr, yarr)

    t = time.time()

    # Maquina de estados
    while True:
        for i in range (16):
            while robot.getDistanceReading(i) <= 2.5:    # Comprobamos si algun sensor detecta un objeto
                robot.Braitenberg()                      # Usamos la funcion Braitenberg para evadir objetos
                print ("Evadiendo obstaculo")
        robot.Trajectory()                               # El robot seguira la trayectoria hasta encontrar un objeto
        print ("Siguiendo Trayectoria")
             
        if (time.time()-t) > END:                        # Tiempo en el que debe terminar la trayectoria
            robot.stop()                                 # Detenemos nuestro robot
            Traject.GrafOut(xt,yt,xorg,yorg)             # Imprimimos resultados
            break

    
    sim.simxGetPingTime(clientID)                        # Desconectamos del Remote Api para finalizar el programa
    sim.simxFinish(clientID)
else:
    print ('Conexion fallida a remote API server')
print ('Fin del programa')
sys.exit('Sin conexion')











