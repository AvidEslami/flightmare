# # Opens a csv file that contains a trajectory and opens a csv file containing a series of points reached on the path
# # Plot the full trajectory and the points reached on the path
# # CSV file format: 
# # t,p_x,p_y,p_z
# # 0.0,10.0,0.0,5.0
# # 0.01,9.99950041666625,0.04997916658333313,5.0
# # ...

# import numpy as np
# import pandas as pd
# import csv
# import matplotlib.pyplot as plt
# import seaborn as sns

# path_to_traj = "./dummy_circle_path.csv"
# path_to_points = "./positions.csv"

# # Plot the trajectory
# sns.set_style("whitegrid")
# path = pd.read_csv(path_to_traj)
# plt.plot(path["p_x"], path["p_y"], label="Trajectory")

# # Plot the points
# points = pd.read_csv(path_to_points)
# plt.scatter(points["x"], points["y"], label="Points", color="red")
# plt.legend()
# plt.xlabel("x")
# plt.ylabel("y")
# plt.title("Trajectory and Points")

# plt.show()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Paths to your CSV files
path_to_traj = "./dummy_circle_path.csv"
path_to_points = "./positions.csv"

# Plot the trajectory
sns.set_style("whitegrid")
path = pd.read_csv(path_to_traj)

# Ensure columns are numeric in case of misreading as strings
path["p_x"] = pd.to_numeric(path["p_x"], errors='coerce')
path["p_y"] = pd.to_numeric(path["p_y"], errors='coerce')

plt.plot(path["p_x"], path["p_y"], label="Trajectory")

# Plot the points
points = pd.read_csv(path_to_points)

# Ensure the points are numeric as well
points["x"] = pd.to_numeric(points["x"], errors='coerce')
points["y"] = pd.to_numeric(points["y"], errors='coerce')

# Scatter plot with larger points for visibility
plt.scatter(points["x"], points["y"], label="Points", color="red", s=1)

# Set axis limits to ensure points are visible within the trajectory's bounds
plt.xlim([path["p_x"].min() - 1, path["p_x"].max() + 1])
plt.ylim([path["p_y"].min() - 1, path["p_y"].max() + 1])

# Plot labels and title
plt.legend()
plt.xlabel("x")
plt.ylabel("y")
plt.title("Trajectory and Points")

# Display the plot
plt.show()
