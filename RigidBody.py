import numpy as np
from pyquaternion import Quaternion
import scipy.constants
import random


class Body(): #Rigid Body 
    def __init__(self, mass, IBody, IBodyInv, X, q, P, L): 
        #constants
        self.mass = mass
        self.IBody = IBody
        self.IBodyInv = IBodyInv
        
        #state variables
        self.X = X
        self.q = q #the quaternion
        self.P = P
        self.L = L
        
        #derived quantities
        self.IInv = np.zeros([3,3])
        self.R = np.asarray([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
        self.v = np.array([0.,0.,0.])
        self.omega = np.array([0.,0.,0.])
        
        #computed quantities
        self.force = np.array([0.,0.,0.])
        self.torque = np.array([0.,0.,0.])
    
    def Compute_Force_and_Torque(self, y, t):
        #planet_position = [0.,0.,0.]
        F = np.array([0.,0.,0.])
        T = np.array([0.,0.,0.])
        
        gravity = False
        if(gravity):
            planet_mass = 1000000000
            r = np.linalg.norm(self.X)
            if(r > 1.):
                r2 = np.power(r,3)
                F = F + np.multiply(-((scipy.constants.gravitational_constant)*planet_mass*self.mass)/r2,self.X)
            else:
                self.P = np.array([0.,0.,0.])
                print(self.X)
                print(self.P)
                exit()

        #state_size = 13 + 8 = 21

        goforward = y[13]
        if(goforward):
            forward = self.q.rotate(np.array([0.,0.,-1.]))
            #forward = np.matmul(self.R, np.array([0.,0.,-1.]))
            thrust = 0.5            
            F = F + (forward * thrust)
            
        gobackward = y[14]
        if(gobackward):
            forward = self.q.rotate(np.array([0.,0.,1.]))
            #forward = np.matmul(self.R, np.array([0.,0.,1.]))
            thrust = 0.5
            F = F + (forward * thrust)
            
        turnUp = y[15]
        if(turnUp):
            thrust = 0.5
            side = self.q.rotate(np.array([-1.,0.,0.]))
            #side = np.matmul(self.R, np.array([1.,0.,0.])) #rolls left
            T = T + (side * thrust)
            
        turnDown = y[16]
        if(turnDown):
            thrust = 0.5
            side = self.q.rotate(np.array([1.,0.,0.])) 
            T = T + (side * thrust)
            
        turnLeft = y[17]
        if(turnLeft):
            thrust = 0.5
            side = self.q.rotate(np.array([0.,-1.,0.])) 
            T = T + (side * thrust)
            
        turnRight = y[18]
        if(turnLeft):
            thrust = 0.5
            side = self.q.rotate(np.array([0.,1.,0.])) 
            T = T + (side * thrust)
            
        rollLeft = y[19]
        if(rollLeft):
            thrust = 0.5
            side = self.q.rotate(np.array([0.,0.,1.])) 
            T = T + (side * thrust)
            
        rollRight = y[20]
        if(rollRight):
            thrust = 0.5
            side = self.q.rotate(np.array([0.,0.,-1.])) 
            T = T + (side * thrust)
        
                
                        
        #gravity now added
        #self.force = np.array([0.01,-0.05,0.]) #random movements for test
        self.force = F
        self.torque = T
        
        

        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            