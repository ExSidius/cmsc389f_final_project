import RigidBody as rb
import Simulator as sm
import outputParse

import numpy as np
from scipy.integrate import odeint
from pyquaternion import Quaternion

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import time

state_size = 13 + 8     #don't change this, default = 13, agentActions = 8
tick_length = 1./30.    #step length between physics checks, don't change this
seconds = 50.           #seconds to simulate
step_size = 1.          #intervals to print seconds at. -1 for no print
verbose = True          #true to print

def mainSim():
    sim = sm.Simulation(state_size) #create simulator
    
    mass = 10. #mass of the object
    dim = np.array([1.,1.,1.]) #dimensions of the cube object
    x = np.array([10.,0.,1.]) #position
    q = Quaternion(np.array([0.,0.,0.,1.])) #rotation
    p = np.array([0.,0.,0.]) #linear momentum
    l = np.array([0.,0.,0.]) #angular momentum
    objectType = "Agent"
    objectName = "Prime"
        #forward, back, up, down, left, right, rollleft, rollright
    thrusts = np.array([50.,0.5 ,0.5,0.5 ,0.5,0.5, 0.5,0.5]) #the thrust magnitude for the various thrusters
    loadtime = 10. #ten second load time between firing projectiles
    
    sim.createObject(mass, dim, x, q, p, l, objectType, objectName, thrusts, loadtime) #add cube object to bodies
    #ATTENTION - OBJECTS MUST HAVE UNIQUE NAMES! 
    #TODO: make it so that objects recieve a unique ID to fix unique name requirement
    
    return sim.runSimulation(tick_length, seconds, step_size, verbose)


time_test = False
if(time_test):
    # DO NOT REPORT THIS... COMPILATION TIME IS INCLUDED IN THE EXECUTION TIME!
    start = time.time()
    mainSim()
    end = time.time()
    print("Elapsed (with compilation) = %s" % (end - start))

    # NOW THE FUNCTION IS COMPILED, RE-TIME IT EXECUTING FROM CACHE
    start = time.time()
    output, entityTracker = mainSim()
    end = time.time()
    print("Elapsed (after compilation) = %s" % (end - start))
else:
    output, entityTracker = mainSim()


#Everything past here is for visualization

maxSamples = 500
if((seconds/tick_length) < maxSamples):
    sampleRate = 1
else:
    sampleRate = int((seconds/tick_length)/maxSamples)

print(sampleRate)
    
    

    
#sampleRate = int((seconds*tick_length) * 4)


v = outputParse.Visualizer(sampleRate)
#                       Name,  Rotation, Color(0,1,2), Line?, Checkpoints
v.addObjectToVisualize("Prime", True, 0, False, -1)
v.addObjectToVisualize("Projectile_0", False, 1, True, 10)
#v.addObjectToVisualize("Projectile_1", False, 1, True)
#v.addObjectToVisualize("Projectile_2", False, 1, True)
#v.addObjectToVisualize("Projectile_3", False, 1, True)
#v.addObjectToVisualize("Projectile_4", False, 1, True)

boxsize = 15.
trim = False
v.visualizeOutput(output, entityTracker, state_size, boxsize, trim)



