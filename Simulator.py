import numpy as np
import RigidBody as rb

from scipy.integrate import ode
import matplotlib.pyplot as plt

import copy

from pyquaternion import Quaternion
from numba import jit

@jit (nopython=True)
def getIInvandOmega(R, IBodyInv, L):
    iinv = np.dot(np.dot(R,IBodyInv),np.transpose(R))
    omega = np.dot(iinv, L)
    return iinv, omega


class Simulation(): #Rigid Body 
    def __init__(self, state_size): 
        self.state_size = state_size
        self.bodies = []
        self.projectileCount = 0
    def State_To_Array(self, state):
        y = 0
        out = np.zeros(self.state_size)
    
        out[y] = state.X[0]; y+=1
        out[y] = state.X[1]; y+=1
        out[y] = state.X[2]; y+=1
    
        out[y] = state.q.real; y+=1
        t = state.q.vector
        out[y] = t[0]; y+=1
        out[y] = t[1]; y+=1
        out[y] = t[2]; y+=1
        
        out[y] = state.P[0]; y+=1
        out[y] = state.P[1]; y+=1
        out[y] = state.P[2]; y+=1
    
        out[y] = state.L[0]; y+=1
        out[y] = state.L[1]; y+=1
        out[y] = state.L[2]; y+=1
    
        return out
    

    def ddt_State_to_Array(self, state):
        y = 0
        out = np.zeros(self.state_size)
        out[y] = state.v[0]; y+=1
        out[y] = state.v[1]; y+=1
        out[y] = state.v[2]; y+=1
        
        omegaq = Quaternion(np.array(np.append([0.],state.omega)))
        qdot = (omegaq * state.q)
        qdot = 0.5 * qdot
    
        out[y] = qdot.real; y+=1
        t = qdot.vector
        out[y] = t[0]; y+=1
        out[y] = t[1]; y+=1
        out[y] = t[2]; y+=1
        
        if(state.alive):
            out[y] = state.force[0]; y+=1
            out[y] = state.force[1]; y+=1
            out[y] = state.force[2]; y+=1
        else:
            out[y] = 0; y+=1
            out[y] = 0; y+=1
            out[y] = 0; y+=1
    
        out[y] = state.torque[0]; y+=1
        out[y] = state.torque[1]; y+=1
        out[y] = state.torque[2]; y+=1
    
        return out
        
    def Array_To_State(self, state, a):
        y = 0
        state.X[0] = a[y]; y+=1
        state.X[1] = a[y]; y+=1
        state.X[2] = a[y]; y+=1
        
        r = a[y]; y+=1
        i = a[y]; y+=1
        j = a[y]; y+=1
        k = a[y]; y+=1
    
        state.q = Quaternion(np.array([r,i,j,k]))
    
        state.P[0] = a[y]; y+=1
        state.P[1] = a[y]; y+=1
        state.P[2] = a[y]; y+=1
    
        state.L[0] = a[y]; y+=1
        state.L[1] = a[y]; y+=1
        state.L[2] = a[y]; y+=1
        
        #compute the auxillary variables
    
        state.v = state.P / state.mass
        state.R = state.q.rotation_matrix
        state.IInv, state.omega = getIInvandOmega(state.R, state.IBodyInv, state.L) 
        #np.matmul(np.matmul(state.R,state.IBodyInv),np.transpose(state.R))
        #state.omega = np.matmul(state.IInv, state.L)
    
        #print(np.reshape(state.L,(3,1)))        
        return state
                    

    def Array_To_Bodies(self, a):
        newBodies = []
        i = 0
        while(i < len(self.bodies)):
            newBodies.append(self.Array_To_State(self.bodies[i],a[(i)*self.state_size:(i+1)*self.state_size]))
            i += 1
        return newBodies
    
    def Bodies_To_Array(self):
        a = np.empty(0)
        i = 0
        while(i < len(self.bodies)):
            a = np.append(a, self.State_To_Array(self.bodies[i]))
            i += 1
        return a
    
    def dydt(self, t, y):
        self.bodies = self.Array_To_Bodies(y)
        ydot = np.empty(0)
        i = 0
        while(i < len(self.bodies)):
            self.bodies[i].Compute_Force_and_Torque(y[i:i+(self.state_size)],t)
            ydot = np.append(ydot, self.ddt_State_to_Array(self.bodies[i]))
            i += 1
        return ydot
        
    def blockIBody(self,x,y,z,M,funmode=False): #dimensions x,y,z of block, mass M
        block = np.array([[(y*y)+(z*z),0.,0.],[0.,(x*x)+(z*z),0.],[0.,0.,(x*x)+(y*y)]])
        if(funmode):
            print("FUNFUNFUNFUNFUNFU")
            block = np.array([[1.,0.,0.],[0.,2.,0.],[0.,0.,3.]])
        Ibody = block * (M/12)  
        return Ibody

    def createObject(self,mass,dim,X,q,P,L, objectType, objectName="unknown", thrusts=np.array([0.,0., 0.,0., 0.,0., 0.,0.]),loadtime=1e+9):
        Ibody = self.blockIBody(dim[0],dim[1],dim[2],mass)
        r = rb.Body(mass, Ibody, np.linalg.inv(Ibody), X, q, P, L, objectType, objectName, thrusts, loadtime)
        self.bodies.append(r)
    
    def addProjectile(self, parent, firespeed):
        mass = 1. #mass of the object
        dim = np.array([0.1,0.1,0.1]) #dimensions of the cube object
        x = parent.X + np.matmul(parent.R, np.array([0.,0.,0.5])) #position
        q = parent.q #rotation
        p = parent.P + np.matmul(parent.R, np.array([0.,0.,firespeed])) #linear momentum
        l = np.array([0.,0.,0.]) #angular momentum
        objectType = "Projectile"
        objectName = "Projectile_{0}".format(self.projectileCount); self.projectileCount += 1
            #forward, back, up, down, left, right, rollleft, rollright
        self.createObject(mass, dim, x, q, p, l, objectType, objectName) #add cube object to bodies
    
    def removeObject(self, y0, name):
        currentSize = y0.shape[0]
        newBodies = []
        index = 0
        i = 0
        while(i < len(self.bodies)):
            if(self.bodies[i].objectName == name):
                index = i
            else:
                newBodies.append(self.bodies[i])
            i += 1
        ynew = np.append(y0[0:(index*self.state_size)],y0[(index+1)*self.state_size:])
        self.bodies = newBodies
        return ynew
        
            
    
    def runSimulation(self, tick_length, seconds, step, verbose):
        output = np.empty(0)
        entityTracker = []
        y0 = np.ones(self.state_size*len(self.bodies))
        yfinal = np.ones(self.state_size*len(self.bodies))
    
        #init states -> initialize the self.bodies!
        yfinal = self.Bodies_To_Array()
        
        
        tt = 0.
        t = 0.
        while(t < seconds): 
            i = 0
            while(i < self.state_size*len(self.bodies)):
                y0[i] = yfinal[i]
                i += 1
                
            output = np.append(output,y0)
            entities = []
            for b in self.bodies:
                entities.append(b.objectName)
            entityTracker.append(entities)
            
            #
            #agent calculations here
            #
                    #forward, back, up, down, left, right, rollleft, rollright
            i = 0
            while(i < len(self.bodies)):
                if(self.bodies[i].alive == False):
                    y0 = self.removeObject(y0, self.bodies[i].objectName)
                    i -= 1
                i += 1
            
            i = 0
            while(i < len(self.bodies)):
                #if(self.bodies[i].alive == False):
                
                if(self.bodies[i].objectType == "Agent"):    #perform AI actions          
                    if(t <= (10*tick_length)):              
                        agentActions = np.array([0.,0., 1.,0., 0.,0., 0.,0.])    
                    elif((t > (10*tick_length)) and (t <= (20*tick_length))):
                        agentActions = np.array([0.,0., 0.,0., 0.,0., 0.,0.])
                    else:
                        agentActions = np.array([0.,0., 0.,0., 0.,0., 0.,0.])
                        
                    if(False):
                        if((t >= tick_length*10) and (t < tick_length*11)):  #shooting a projectile
                            if(verbose):
                                print("FIRE FIRE FIRE")
                            self.addProjectile(self.bodies[i], 0.005)
                            y0 = np.append(y0, self.State_To_Array(self.bodies[-1]))
                            yfinal = y0
                        if((t >= tick_length*21) and (t < tick_length*22)):  #shooting a projectile
                            if(verbose):
                                print("FIRE FIRE FIRE")
                            self.addProjectile(self.bodies[i], 0.01)
                            y0 = np.append(y0, self.State_To_Array(self.bodies[-1]))
                            yfinal = y0
                        if((t >= tick_length*33) and (t < tick_length*34)):  #shooting a projectile
                            if(verbose):
                                print("FIRE FIRE FIRE")
                            self.addProjectile(self.bodies[i], 0.02)
                            y0 = np.append(y0, self.State_To_Array(self.bodies[-1]))
                            yfinal = y0
                        if((t >= tick_length*45) and (t < tick_length*46)):  #shooting a projectile
                            if(verbose):
                                print("FIRE FIRE FIRE")
                            self.addProjectile(self.bodies[i], 0.03)
                            y0 = np.append(y0, self.State_To_Array(self.bodies[-1]))
                            yfinal = y0
                    if((t >= tick_length*57) and (t < tick_length*58)):  #shooting a projectile
                        if(verbose):
                            print("FIRE FIRE FIRE")
                        self.addProjectile(self.bodies[i], 0.04)
                        y0 = np.append(y0, self.State_To_Array(self.bodies[-1]))
                        yfinal = y0
                else: #object has no actions
                    agentActions = np.array([0.,0., 0.,0., 0.,0., 0.,0.])
                    
                y0[(i*self.state_size)+13:(i*self.state_size)+21] = agentActions
                i += 1
            
            #u = []
            #tlist = []
            
            r = ode(self.dydt).set_integrator('dop853', rtol=0., atol=1e-9,nsteps=10)
            
            #r = ode(self.dydt).set_integrator('vode', method='adams',
            #                        order=10, rtol=0., atol=1e-9,
            #                        with_jacobian=False, nsteps=10)
            
            
            r.set_initial_value(y0,0)
            
            #while(r.successful() and r.t < tick_length): #do one integration
                #u.append(r.y); tlist.append(r.t)
                #yfinal = (odeint(self.dydt,y0,[t, t + tick_length]))[1]
                
            r.integrate(r.t + tick_length)
            yfinal = r.y
            
            if(step != -1 and tt >= step):
                if(verbose):
                    print("seconds: {0}".format(t))
                    if(len(self.bodies) > 1):
                        body = 1
                        print("X: ", self.bodies[body].X)
                        print("Q: ", self.bodies[body].q)
                        print("P: ", self.bodies[body].P)
                        print("L: ", self.bodies[body].L)
                        print("tau: ", self.bodies[body].torque)
                tt = 0.
                
            t += tick_length
            tt += tick_length
        return output, entityTracker










