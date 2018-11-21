import numpy as np
from pyquaternion import Quaternion
import scipy.constants
import random


class Body(): #Rigid Body 
    def __init__(self, mass, IBody, IBodyInv, X, q, P, L, objectType, objectName, thrusts, loadtime): 
        #constants
        self.mass = mass
        self.IBody = IBody
        self.IBodyInv = IBodyInv
        
        self.thrusts = thrusts
        self.loadtime = loadtime
        
        self.objectType = objectType
        self.objectName = objectName
        
        #state variables
        self.X = X
        self.q = q #the quaternion
        self.P = P
        self.L = L
        
        self.lastshot = -(loadtime + 1.) #start out being able to fire
        
        #derived quantities
        self.IInv = np.zeros([3,3])
        self.R = np.asarray([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
        self.v = np.array([0.,0.,0.])
        self.omega = np.array([0.,0.,0.])
        
        #computed quantities
        self.force = np.array([0.,0.,0.])
        self.torque = np.array([0.,0.,0.])
        self.alive = True
        
    def checkFire(self, t):
        #check and see if the agent can fire
        if(t - self.lastshot > self.loadtime):
            return True
    
    def Compute_Force_and_Torque(self, y, t):
        planet_position = np.array([0.,0.,0.])
        F = np.array([0.,0.,0.])
        T = np.array([0.,0.,0.])
        
        gravity = True
        planetRadius = 1.
        if(gravity and self.alive):
            planet_mass = 10000000000
            
            collision = False
            
            p1 = self.X
            p2 = self.X + (self.P/self.mass)
            pA = planet_position
            
            directionP = p2 - p1
            tempB = p1 - pA
            
            t = -(np.sum(directionP*tempB))/np.sum(directionP*directionP)
            X = p1 + (t*directionP)
            dist1 = np.linalg.norm(X - p1)
            dist2 = np.linalg.norm(X - p2)
            dist3 = np.linalg.norm(p1 - p2)
            
            tdist = dist1 + dist2
            e = 0.0001
            if(tdist >= dist3 - e and tdist <= dist3 + e):
                AX = tempB + (t*directionP)
                minDistance = np.linalg.norm(AX)
                if(minDistance <= planetRadius):
                    collision = True
                        
            if(not collision):
                r = np.linalg.norm(self.X)
                r2 = np.power(r,3)
                F = F + np.multiply(-((scipy.constants.gravitational_constant)*planet_mass*self.mass)/r2,self.X)
            else:
                #F = F + np.matmul(self.R, -self.P)
                self.alive = False
                print("hit planet")            

        #state_size = 13 + 8 = 21
       

        if(self.objectType == "Agent" and self.alive):
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
                thrust = self.thrusts[2] 
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
                thrust = self.thrusts[3] 
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
                thrust = self.thrusts[4] 
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
                thrust = self.thrusts[5] 
                l1 = np.array([-0.5,0.,0.5])   #location of point one
                l2 = np.array([0.5,0.,-0.5]) 
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
            
            rollLeft = y[19]
            if(rollLeft):
                thrust = self.thrusts[6] 
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
                thrust = self.thrusts[7] 
                l1 = np.array([-0.5,0.5,0.])   #location of point one
                l2 = np.array([0.5,-0.5,0.]) 
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
                        
        self.force = F
        self.torque = T
        
        

        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            