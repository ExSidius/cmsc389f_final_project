import numpy as np
import RigidBody as rb

from scipy.integrate import odeint
import matplotlib.pyplot as plt

from pyquaternion import Quaternion


state_size = 13
bodies = []


def State_To_Array(state):
    y = 0
    out = np.zeros(state_size)
    
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
    
def Star(a):
    t = np.asarray([[0.,-a[2],a[1]],[a[2],0.,-a[0]],[-a[1],a[0],0.]])
    return t

def ddt_State_to_Array(state):
    y = 0
    out = np.zeros(state_size)
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
        
def Array_To_State(state, a):
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
    state.IInv = np.matmul(np.matmul(state.R,state.IBodyInv),np.transpose(state.R))
    state.omega = np.matmul(state.IInv, state.L)
    state.R = state.q.rotation_matrix
    
    #print(np.reshape(state.L,(3,1)))        
    return state
                    

def Array_To_Bodies(a):
    newBodies = []
    i = 0
    while(i < len(bodies)):
        newBodies.append(Array_To_State(bodies[i],a[(i)*state_size:(i+1)*state_size]))
        i += 1
    return newBodies
    
def Bodies_To_Array():
    a = np.empty(0)
    i = 0
    while(i < len(bodies)):
        a = np.append(a, State_To_Array(bodies[i]))
        i += 1
    return a
    
def dydt(y, t):
    bodies = Array_To_Bodies(y)
    ydot = np.empty(0)
    i = 0
    while(i < len(bodies)):
        bodies[i].Compute_Force_and_Torque(t)
        ydot = np.append(ydot, ddt_State_to_Array(bodies[i]))
        i += 1
    return ydot
    
    
def runSimulation(bodies, seconds):
    y0 = np.ones(state_size*len(bodies))
    yfinal = np.ones(state_size*len(bodies))
    
    #init states -> initialize the bodies!
    yfinal = Bodies_To_Array()
    
    t = 0.
    while(t < seconds): 
        i = 0
        while(i < state_size*len(bodies)):
            y0[i] = yfinal[i]
            i += 1
        yfinal = (odeint(dydt,y0,[t, t + 1./30.]))[1]
        #ode(y0, yfinal, STATE_SIZE * NBODIES, t, t+1./30., dydt);
        t += 1./30.
    return bodies


def blockIBody(x,y,z,M): #dimensions x,y,z of block, mass M
    block = np.array([[(y*y)+(z*z),0.,0.],[0.,(x*x)+(z*z),0.],[0.,0.,(x*x)+(y*y)]])
    Ibody = block * (M/12)
    #intertia tensor made
    return Ibody

def createObject(mass,dim,X,R,P=np.array([0.,0.,0.]),L=np.array([0.,0.,0.])):
    Ibody = blockIBody(dim[0],dim[1],dim[2],mass)
    r = rb.Body(mass, Ibody, np.linalg.inv(Ibody), X, R, P, L)
    return r
    #    def __init__(self, mass, IBody, IBodyInv, X, R, P, L):


#state size is 18

x = np.array([0.,0.5,0.])
q = Quaternion(np.array([0.,1.,0.,0.]))
p = np.array([0.,0.,0.])
l = np.array([0.,0.,0.])
one = createObject(10., np.array([10.,1.,10.]), x, q, p, l)
bodies.append(one)

print(bodies[0].X)
print(bodies[0].q)

i = 0
while(i < 2): #1000 seconds
    bodies = runSimulation(bodies, 10)
    print("{0} seconds".format(i*10))
    i += 1

print(bodies[0].X)
print(bodies[0].q)









