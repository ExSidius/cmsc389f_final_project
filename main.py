import RigidBody as rb
import Simulator as sm

import numpy as np
from scipy.integrate import odeint
from pyquaternion import Quaternion

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt




state_size = 13 + 8#don't change this, default = 13
sim = sm.Simulation(state_size) #create simulator

mass = 10. #mass of the object
dim = np.array([1.,1.,1.]) #dimensions of the cube object
x = np.array([0.,0.,0.]) #position
q = Quaternion(np.array([0.,0.,0.,1.])) #rotation
p = np.array([0.,0.,50.]) #linear momentum
l = np.array([0.,5.,0.]) #angular momentum
    #forward, back, up, down, left, right, rollleft, rollright
thrusts = np.array([5.,0.5 ,0.5,0.5 ,0.5,0.5, 0.5,0.5]) #the thrust magnitude for the various thrusters
sim.createObject(mass, dim, x, q, p, l, thrusts) #add cube object to bodies

tick_length = 1./30.  #step length between physics checks, don't change this
seconds = 30.     #seconds to simulate
step_size = 1.        # intervals to print seconds at. -1 for no print

output = sim.runSimulation(tick_length, seconds, step_size)


#Everything past here is for visualization

Z_line = np.ones((output.shape[0],3))
Y_line = np.ones((output.shape[0],3))
X_line = np.ones((output.shape[0],3))

r = output[:,3:7]
i = 0
while(i < output.shape[0]):
    tq = Quaternion(r[i])
    trZ = tq.rotate(np.array([0.,0.,1.]))
    trY = tq.rotate(np.array([0.,1.,0.]))
    trX = tq.rotate(np.array([1.,0.,0.]))
    Z_line[i] = output[i,0:3] + trZ
    Y_line[i] = output[i,0:3] + trY
    X_line[i] = output[i,0:3] + trX
    i += 1

mpl.rcParams['legend.fontsize'] = 8

#print(output[output.shape[0]-1,:])
#print(output[:,2])
#exit()

#output = output[3100:3300]
#temp = temp[3100:3300]

fig = plt.figure()

ax = fig.gca(projection='3d')


if(True):
    samplemodifier = 4
    samplesize = int(step_size/tick_length) * samplemodifier
    output = output[0::samplesize]
    print(output.shape[0])
    colors = np.zeros((output.shape[0],3))
    
    i = 0
    while(i < colors.shape[0]):
        colors[i,0] = i / colors.shape[0]
        colors[i,1] = 0
        colors[i,2] = 0
        i += 1

    ax.scatter(output[:,0], output[:,1], output[:,2], label='X, every {0} seconds'.format(step_size*samplemodifier), color=colors)

    f = X_line.shape[0]
    s = output[output.shape[0]-1,0:3]
    e = X_line[f-1]
    se = np.vstack((s,e))

    h1 = output[0,0:3]
    h2 = X_line[0]
    h3 = np.vstack((h1,h2))

    ax.plot(X_line[:,0], X_line[:,1], X_line[:,2], label='X+rot*[1,0,0]   X', color=(1,0,0))
    ax.scatter(X_line[0,0], X_line[0,1], X_line[0,2], color='r', marker='o')                  #circle = start
    ax.scatter(X_line[f-1:f,0], X_line[f-1:f,1], X_line[f-1:f,2], color='r', marker='^')      #triangle = end
    ax.plot(se[:,0],se[:,1],se[:,2], color = 'k') #line between last position and the last X,Y,or Z marker
    ax.plot(h3[:,0],h3[:,1],h3[:,2], color = (1.,1.,.4)) #line between the first position and the first X,Y,or Z marker

    f = Y_line.shape[0]
    s = output[output.shape[0]-1,0:3]
    e = Y_line[f-1]
    se = np.vstack((s,e))
    h1 = output[0,0:3]
    h2 = Y_line[0]
    h3 = np.vstack((h1,h2))

    f = Y_line.shape[0]
    ax.plot(Y_line[:,0], Y_line[:,1], Y_line[:,2], label='X+rot*[0,1,0]   Y', color=(0,1,0))
    ax.scatter(Y_line[0,0], Y_line[0,1], Y_line[0,2], color='r', marker='o')                  #circle = start
    ax.scatter(Y_line[f-1:f,0], Y_line[f-1:f,1], Y_line[f-1:f,2], color='r', marker='^')      #triangle = end
    ax.plot(se[:,0],se[:,1],se[:,2], color = 'k') #line between last position and the last X,Y,or Z marker
    ax.plot(h3[:,0],h3[:,1],h3[:,2], color = (1.,1.,.4)) #line between the first position and the first X,Y,or Z marker

    f = Z_line.shape[0]
    s = output[output.shape[0]-1,0:3]
    e = Z_line[f-1]
    se = np.vstack((s,e))
    h1 = output[0,0:3]
    h2 = Z_line[0]
    h3 = np.vstack((h1,h2))

    f = Z_line.shape[0]
    ax.plot(Z_line[:,0], Z_line[:,1], Z_line[:,2], label='X+rot*[0,0,1]   Z', color=(0,0,1))
    ax.scatter(Z_line[0,0], Z_line[0,1], Z_line[0,2], color='r', marker='o')                  #circle = start
    ax.scatter(Z_line[f-1:f,0], Z_line[f-1:f,1], Z_line[f-1:f,2], color='r', marker='^')      #triangle = end
    ax.plot(se[:,0],se[:,1],se[:,2], color = 'k') #line between last position and the last X,Y,or Z marker
    ax.plot(h3[:,0],h3[:,1],h3[:,2], color = (1.,1.,.4)) #line between the first position and the first X,Y,or Z marker
    
    ax.legend()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
else:
    ax.plot(output[:,21], output[:,24], output[:,22], label='tau v time', color='g')
    ax.legend()
    ax.set_xlabel("X")
    ax.set_ylabel("time")
    ax.set_zlabel("Y")
    


plt.show()
