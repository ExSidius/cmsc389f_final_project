import numpy as np
from pyquaternion import Quaternion


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
    
    def Compute_Force_and_Torque(self, t):
        self.force = np.array([0.01,-0.05,0.]) #random movements for test
        self.torque = np.array([0.01,0.,0.01])
        

        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            