import matplotlib.pyplot as plt
import numpy as np

data_folder = "/home/artin/Desktop/flightmare/data/velocities.txt"

# data has the following format 
#   diffx diffy diffz
#   goalx goaly goalz
#   currentx currenty currentz
#   diffx diffy diffz
#   goalx goaly goalz
#   currentx currenty currentz
# ...

data = np.loadtxt(data_folder)

# We would like to plot the x, y, z velocities of diff, goal, and current on three different plots
fig, axs = plt.subplots(3, 1)

axs[0].plot(data[0::3, 0], label='diffx')
axs[0].plot(data[0::3, 1], label='diffy')
axs[0].plot(data[0::3, 2], label='diffz')
axs[0].set(xlabel='time', ylabel='diff',
       title='Goal/Observed velocity')
axs[0].grid()
axs[0].legend()

axs[1].plot(data[1::3, 0], label='goalx')
axs[1].plot(data[1::3, 1], label='goaly')
axs[1].plot(data[1::3, 2], label='goalz')
axs[1].set(xlabel='time', ylabel='goal',
       title='Goal Velocity')
axs[1].grid()
axs[1].legend()

axs[2].plot(data[2::3, 0], label='currentx')
axs[2].plot(data[2::3, 1], label='currenty')
axs[2].plot(data[2::3, 2], label='currentz')
axs[2].set(xlabel='time', ylabel='current',
       title='Observed Velocity')
axs[2].grid()
axs[2].legend()

# space out the plots
plt.tight_layout()

plt.show()