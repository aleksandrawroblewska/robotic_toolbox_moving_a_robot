import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import *
import math as m

robot = rtb.models.Panda()
via_pt = []
for t in range(0, 24):
    x = 0.1 * m.cos(t * np.pi / 12)+0.65
    y = 0.1 * m.sin(t * np.pi / 12)+0.2
    macierz = SE3(x,y,0.15)*SE3.Ry(np.pi)
    iksolution = robot.ikine_LMS(macierz)
    via_pt.append(iksolution.q)
    plt.scatter(x,y)
plt.show()
print(iksolution.q)
traj = mstraj(np.asarray(via_pt), dt=0.1, tacc=0.2, qdmax=2.0)
robot.plot(traj.q, backend = 'pyplot', movie='panda_pyplot.gif')