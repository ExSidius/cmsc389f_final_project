import numpy as np
from pyquaternion import Quaternion
import scipy.constants
import random


class Body(): #Rigid Body 
    def __init__(self, mass, IBody, IBodyInv, X, q, P, L, thrusts): 
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
        
        self.thrusts = thrusts
    
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
            thrust = self.thrusts[0]     
            forward = np.matmul(self.R, np.array([0.,0.,1.]))     
            #forward = self.q.rotate(np.array([0.,0.,1.]))
            F = F + (forward * thrust)
            
        gobackward = y[14]
        if(gobackward):
            thrust = self.thrusts[1] 
            forward = np.matmul(self.R, np.array([0.,0.,-1.]))
            #forward = self.q.rotate(np.array([0.,0.,-1.]))
            F = F + (forward * thrust)
            
        turnUp = y[15]
        if(turnUp):
            self.thrusts[2] 
            l1 = np.array([0.,-0.5,0.5])   #location of point one
            l2 = np.array([0.,0.5,-0.5]) 
            f1 = np.array([0.,0.,-thrust]) #force one on point one
            f2 = np.array([0.,0.,thrust])
            
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
            
            #f1 = self.q.rotate(f1)
            #f2 = self.q.rotate(f2)
            
            #rl1 = self.q.rotate(l1)
            #rl2 = self.q.rotate(l2)
             
            rlf1 = np.cross(rl1, f1)
            rlf2 = np.cross(rl2, f2)
            
            tau = rlf1 + rlf2
            T = T + tau
            
        turnDown = y[16]
        if(turnDown):
            self.thrusts[3] 
            l1 = np.array([0.,0.5,0.5])   #location of point one
            l2 = np.array([0.,-0.5,-0.5]) 
            f1 = np.array([0.,0.,-thrust]) #force one on point one
            f2 = np.array([0.,0.,thrust])
            
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
             
            rlf1 = np.cross(rl1, f1)
            rlf2 = np.cross(rl2, f2)
            
            tau = rlf1 + rlf2
            T = T + tau
            
        turnLeft = y[17]
        if(turnLeft):
            self.thrusts[4] 
            l1 = np.array([0.5,0.,0.5])   #location of point one
            l2 = np.array([-0.5,0.,-0.5]) 
            f1 = np.array([0.,0.,-thrust]) #force one on point one
            f2 = np.array([0.,0.,thrust])
            
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
             
            rlf1 = np.cross(rl1, f1)
            rlf2 = np.cross(rl2, f2)
            
            tau = rlf1 + rlf2
            T = T + tau
            
        turnRight = y[18]   
        if(turnRight):
            self.thrusts[5] 
            l1 = np.array([-0.5,0.,0.5])   #location of point one
            l2 = np.array([0.5,0.,-0.5]) 
            f1 = np.array([0.,0.,-thrust]) #force one on point one
            f2 = np.array([0.,0.,thrust])
                    
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
             
            rlf1 = np.cross(rl1, rf1)
            rlf2 = np.cross(rl2, rf2)
            
            tau = rlf1 + rlf2
            T = T + tau
            
        rollLeft = y[19]
        if(rollLeft):
            self.thrusts[6] 
            l1 = np.array([0.5,0.5,0.])   #location of point one
            l2 = np.array([-0.5,-0.5,0.]) 
            f1 = np.array([0.,-thrust,0.]) #force one on point one
            f2 = np.array([0.,thrust,0.])
            
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
             
            rlf1 = np.cross(rl1, f1)
            rlf2 = np.cross(rl2, f2)
            
            tau = rlf1 + rlf2
            T = T + tau
            
        rollRight = y[20]
        if(rollRight):
            self.thrusts[7] 
            l1 = np.array([-0.5,0.5,0.])   #location of point one
            l2 = np.array([0.5,-0.5,0.]) 
            f1 = np.array([0.,-thrust,0.]) #force one on point one
            f2 = np.array([0.,thrust,0.])
            
            f1 = np.matmul(self.R, f1)
            f2 = np.matmul(self.R, f2)
            
            rl1 = np.matmul(self.R, l1)
            rl2 = np.matmul(self.R, l2)
            
            #f1 = self.q.rotate(f1)
            #f2 = self.q.rotate(f2)
            
            #rl1 = self.q.rotate(l1)
            #rl2 = self.q.rotate(l2)
            
            rlf1 = np.cross(rl1, f1)
            rlf2 = np.cross(rl2, f2)
            
            tau = rlf1 + rlf2
            T = T + tau
        if(False):
            T = T + np.array([0.,0.,0.0])
        if(False):
            T = T + np.array([0.,0.,0.1])
        
                
                        
        #gravity now added
        #self.force = np.array([0.01,-0.05,0.]) #random movements for test
        self.force = F
        self.torque = T
        
        

        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            