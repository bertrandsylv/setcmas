# -*- coding: utf-8 -*-
"""
(c) Sylvain Bertrand, 2023
"""


import numpy as np
import matplotlib.pyplot as plt
import sys



if (len(sys.argv) == 2):
    nbOfRobots = int(sys.argv[1])
else:
    nbOfRobots = 1 
    print("Warning: nb of robots set to 1")
    print("syntax: plot_from_csv.py nbOfRobots")


t = []
x = []
y = []
trig_evt = []

fig, ax = plt.subplots(2)
fig_traj, ax_traj = plt.subplots(1)
fig_evt, ax_evt = plt.subplots(nbOfRobots)

colorList = [[1,0,0], [0,1,0], [0,0,1], [1,0.91,0.31], [0.7,0.7,0.7], [0.96,0.18,0.92], [1,0.49,0.37], [0,0,0]]
while len(colorList)<nbOfRobots:
    colorList.append([0.5,0.5,0.5])

legendList = []


for robotNo in range(1,nbOfRobots+1):

    # load CSV file
    fileName = 'csv_directory/_slash_tb3_'+str(robotNo)+'_slash_odom.csv'
    f = np.loadtxt(fileName, skiprows=1, usecols=[4,5,11,12,13], delimiter=',')
    
    # get fields
    t.append(f[:,0]-f[0,0]+f[:,1]*0.000000001)
    x.append(f[:,2])
    y.append(f[:,3])


    # load CSV file
    fileNameEvtTrig = 'csv_directory/_slash_tb3_'+str(robotNo)+'_slash_trig_event.csv'
    fEvtTrig = np.loadtxt(fileNameEvtTrig, skiprows=1, usecols=[0,1], delimiter=',')
    trig_evt.append(fEvtTrig[:,1])

    
    legendList.append(str(robotNo))
    

    # time plots
    ax[0].plot(t[robotNo-1], x[robotNo-1], color=colorList[robotNo-1])
    ax[1].plot(t[robotNo-1], y[robotNo-1], color=colorList[robotNo-1])

    # trajectory plot
    ax_traj.plot(x[robotNo-1],y[robotNo-1], color=colorList[robotNo-1])
    
    # triggering events plot
    if (nbOfRobots>1):
        ax_evt_c = ax_evt[robotNo-1]
    else:
        ax_evt_c = ax_evt
    ax_evt_c.plot(trig_evt[robotNo-1], color=colorList[robotNo-1])
    ax_evt_c.set_ylabel('robot '+str(robotNo))
    ax_evt_c.grid()
    if (robotNo==1):
        ax_evt_c.set_title('Communication triggering events')


# legend and labels for time plots
ax[0].set_ylabel('x (m)')
ax[0].grid()
ax[0].legend(legendList)
ax[1].set_xlabel('t (s)')
ax[1].set_ylabel('y (m)')
ax[1].grid()
ax[1].legend(legendList)

# legend and labelds for trajectory plot
ax_traj.set_title('Trajectories')
ax_traj.set_xlabel('x (m)')
ax_traj.set_ylabel('y (m)')
ax_traj.grid()
ax_traj.legend(legendList)

# show plots
plt.show()
