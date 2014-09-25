import pylab as pl
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import mpl
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
import sg_filter

f = open('kuka_task_error.txt')
lines = f.readlines()
f.close()

errEE=[float(x) for x in lines[0].split()]
errEL=[float(x) for x in lines[1].split()]
errQ2=[float(x) for x in lines[2].split()]
time=[float(x) for x in lines[3].split()]

for i in range(len(errQ2)):
  errQ2[i]=errQ2[i]*0.5

errQ = np.asarray(errQ2)


errQ = sg_filter.smooth1d(errQ, 101)
  
dt = 0.001
#switch_times = [250,9000,16000,23000]
#switch_finish_times = [2250,11000,18000,25000]

switch_times = [250,15000,33000,49000]
switch_finish_times = [2250,17000,35000,51000]

pl.figure()

pl.plot(time,errEE, 'r', label ='end-effector')
pl.plot(time,errEL, 'b', label = 'elbow', linewidth = 3)
pl.plot(time[35000:49000],errEL[35000:49000], color='#3399ff', linewidth = 3)
pl.plot(time,errQ, 'k--',label = 'posture')
p = pl.axvspan(switch_times[0]*dt, switch_finish_times[0]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[1]*dt, switch_finish_times[1]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[2]*dt, switch_finish_times[2]*dt, facecolor='k', alpha=0.2)
p = pl.axvspan(switch_times[3]*dt, switch_finish_times[3]*dt, facecolor='k', alpha=0.2)

pl.ylabel('task error')
pl.xlim(0.0,63.0)
pl.ylim(-0.01,0.8)
pl.legend(loc = 'best')
pl.grid()

pl.show()
