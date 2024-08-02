import matplotlib.pyplot as plt
import numpy as np

data_folder = "/home/artin/Desktop/flightmare/data/orientations.txt"

# data has the following format 
#   diffz diffy diffx
#   goalz goaly goalx
#   currentz currenty currentx
#   diffz diffy diffx
#   goalz goaly goalx
#   currentz currenty currentx
# ...

data = np.loadtxt(data_folder)

# We would like to plot the x, y, z orientations of diff, goal, and current on diff plots, x on top, y in the middle, z on the bottom

fig, axs = plt.subplots(4, 1)

# show average orientation change
avg_x = np.mean(data[0::3, 2])
avg_y = np.mean(data[0::3, 1])
avg_z = np.mean(data[0::3, 0])

# round average to 3 decimal points
avg_x = round(avg_x, 3)
avg_y = round(avg_y, 3)
avg_z = round(avg_z, 3)

axs[0].plot(data[0::3, 2], label='diffx')
axs[0].plot(data[0::3, 1], label='diffy')
axs[0].plot(data[0::3, 0], label='diffz')

axs[0].set(xlabel='time', ylabel='diff',
                title='Goal/Observed orientation')
axs[0].grid()
axs[0].legend()

# graphing x goal vs x current
axs[1].plot(data[1::3, 2], label='goalx')
axs[1].plot(data[2::3, 2], label='currentx')

axs[1].set(xlabel='time', ylabel='goal/current',
                title='Goal/Current orientation X, avg diff: {}'.format(avg_x))

axs[1].grid()
axs[1].legend()

# graphing y goal vs y current
axs[2].plot(data[1::3, 1], label='goaly')
axs[2].plot(data[2::3, 1], label='currenty')

axs[2].set(xlabel='time', ylabel='goal/current',
                title='Goal/Current orientation Y, avg diff: {}'.format(avg_y))

axs[2].grid()
axs[2].legend()

# graphing z goal vs z current
axs[3].plot(data[1::3, 0], label='goalz')
axs[3].plot(data[2::3, 0], label='currentz')

axs[3].set(xlabel='time', ylabel='goal/current',
                title='Goal/Current orientation Z, avg diff: {}'.format(avg_z))

axs[3].grid()
axs[3].legend()

# space out the plots
plt.tight_layout()
plt.show()