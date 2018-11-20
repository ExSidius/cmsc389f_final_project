import RigidBody as rb
import Simulator as sm
import outputParse

import numpy as np
from scipy.integrate import odeint
from pyquaternion import Quaternion

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


state_size = 13 + 8 #don't change this, default = 13, agentActions = 8
sim = sm.Simulation(state_size) #create simulator

mass = 10. #mass of the object
dim = np.array([1.,1.,1.]) #dimensions of the cube object
x = np.array([0.,0.,10.]) #position
q = Quaternion(np.array([0.,0.,0.,1.])) #rotation
p = np.array([0.,0.,0.5]) #linear momentum
l = np.array([0.,0.,0.]) #angular momentum
objectType = "Agent"
objectName = "Prime"
    #forward, back, up, down, left, right, rollleft, rollright
thrusts = np.array([50.,0.5 ,0.5,0.5 ,0.5,0.5, 0.5,0.5]) #the thrust magnitude for the various thrusters
loadtime = 10. #ten second load time between firing projectiles
sim.createObject(mass, dim, x, q, p, l, objectType, objectName, thrusts, loadtime) #add cube object to bodies

#ATTENTION - OBJECTS MUST HAVE UNIQUE NAMES! 
#TODO: make it so that objects recieve a unique ID to fix unique name requirement

tick_length = 1./30.  #step length between physics checks, don't change this
seconds = 30.     #seconds to simulate
step_size = 1.        # intervals to print seconds at. -1 for no print

output, entityTracker = sim.runSimulation(tick_length, seconds, step_size)


#Everything past here is for visualization

sampleRateModifier = 4
sampleRate = int(step_size/tick_length) * sampleRateModifier

v = outputParse.Visualizer(sampleRate)
v.addObjectToVisualize("Prime", True, 0)
v.addObjectToVisualize("Projectile_0", False, 1)
v.visualizeOutput(output, entityTracker, state_size)



