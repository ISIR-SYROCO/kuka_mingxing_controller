import pylab as pl
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import mpl
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure

f = open('kuka_task_error.txt')
lines = f.readlines()
f.close()

errEE=[float(x) for x in lines[0].split()]
errEL=[float(x) for x in lines[1].split()]
errQ=[float(x) for x in lines[2].split()]
time=[float(x) for x in lines[3].split()]

for i in range(len(errQ)):
  errQ[i]=errQ[i]*0.5

  
dt = 0.001
switch_times = [250,9000,16000,23000]
switch_finish_times = [2250,11000,18000,25000]
pl.figure()

pl.plot(time,errEE, 'r', label ='end-effector', linewidth = 3)
pl.plot(time,errEL, 'b', label = 'elbow')
pl.plot(time,errQ, 'k--',label = 'posture')
p = pl.axvspan(switch_times[0]*dt, switch_finish_times[0]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[1]*dt, switch_finish_times[1]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[2]*dt, switch_finish_times[2]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[3]*dt, switch_finish_times[3]*dt, facecolor='k', alpha=0.2)

pl.ylabel('task error')
pl.ylim(-0.01,0.8)
pl.legend(loc = 'best')
pl.grid()

pl.show()
