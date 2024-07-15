import pandas as pd

# We would like to plot the progress of the training of the model "true_reward"
file_path = "/home/artin/Desktop/flightmare/flightrl/examples/successes/2024-03-24-01-20-43/progress.csv"
data = pd.read_csv(file_path)

# Data has the following format
# time_elapsed,clipfrac,policy_loss,n_updates,ep_len_mean,value_loss,approxkl,explained_variance,policy_entropy,serial_timesteps,true_reward,true_reward,total_timesteps,fps

# We would like to plot the progress of the training of the model "true_reward"
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
ax.plot(data["time_elapsed"], data["true_reward"])
ax.set(xlabel='time_elapsed', ylabel='true_reward',
       title='true_reward')
ax.grid()
plt.show()
