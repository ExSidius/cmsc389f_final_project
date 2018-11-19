import numpy as np
import RigidBody as rb

from scipy.integrate import ode
import matplotlib.pyplot as plt

from pyquaternion import Quaternion


class Simulation(): #Rigid Body 
    def __init__(self, state_size): 
        self.state_size = state_size
        self.bodies = []

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
    
    def Star(self, a):
        t = np.asarray([[0.,-a[2],a[1]],[a[2],0.,-a[0]],[-a[1],a[0],0.]])
        return t

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
        
        out[y] = state.force[0]; y+=1
        out[y] = state.force[1]; y+=1
        out[y] = state.force[2]; y+=1
    
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
        state.IInv = np.matmul(np.matmul(state.R,state.IBodyInv),np.transpose(state.R))
        state.omega = np.matmul(state.IInv, state.L)
    
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
    
    
    def runSimulation(self, tick_length, seconds, step):
        output = np.zeros(self.state_size)
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
            output = np.vstack((output,y0))
            
            #
            #agent calculations here
            #
                    #forward, back, up, down, left, right, rollleft, rollright
            agentActions = np.array([0.,0., 0.,0., 1.,0., 0.,0.])
            y0[13:21] = agentActions
            #u = []
            #tlist = []
            
            r = ode(self.dydt).set_integrator('vode', method='adams',
                                    order=10, rtol=0., atol=1e-9,
                                    with_jacobian=False, nsteps=10)
            
            r.set_initial_value(y0,0)
            
            #while(r.successful() and r.t < tick_length): #do one integration
                #u.append(r.y); tlist.append(r.t)
                #yfinal = (odeint(self.dydt,y0,[t, t + tick_length]))[1]
                
            r.integrate(r.t + tick_length)
            yfinal = r.y
            
            if(step != -1 and tt >= step):
                print("seconds: {0}".format(t))
                print("X: ", self.bodies[0].X)
                print("Q: ", self.bodies[0].q)
                print("L: ", self.bodies[0].P)
                tt = 0.
                
            t += tick_length
            tt += tick_length
        return output[1:]

    def blockIBody(self,x,y,z,M): #dimensions x,y,z of block, mass M
        block = np.array([[(y*y)+(z*z),0.,0.],[0.,(x*x)+(z*z),0.],[0.,0.,(x*x)+(y*y)]])
        Ibody = block * (M/12)
        #intertia tensor made
        return Ibody

    def createObject(self,mass,dim,X,q,P,L):
        Ibody = self.blockIBody(dim[0],dim[1],dim[2],mass)
        r = rb.Body(mass, Ibody, np.linalg.inv(Ibody), X, q, P, L)
        self.bodies.append(r)








