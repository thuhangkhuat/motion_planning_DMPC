from multi_robot_plot import plot_robot_and_obstacles
from create_obstacles import create_obstacles
from Drone import Drone

import numpy as np
import time
import matplotlib.pyplot as plt

SIM_TIME = 50
TIMESTEP = 0.2
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.4
VMAX = 2
VMIN = 0.2
VREF = 1.0
OBS_RADIUS = 0.7
obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)
starts = np.array([[1,1],[1,3],[1,5],[1,7],[1,9],[34,9],[34,7],[34,5],[34,3],[34,1]])
goals = np.array([[34,9],[34,7],[34,5],[34,3],[34,1],[1,1],[1,3],[1,5],[1,7],[1,9]])
num_robot = starts.shape[0]
d = 2   # Distance between two consecutive robot
alpha = 3*np.pi/4 # Angle

drones = []
# Initialize Drone
for i in range(starts.shape[0]):
    drone = Drone(i, starts[i,:], np.array([0,0]), ROBOT_RADIUS)
    drones.append(drone)

filename = ""

# for i in range(NUMBER_OF_TIMESTEPS):
i = 0
epsilon = 0.1
ts = []
robot_state_history = []
robot_control_history = []

while np.any(np.linalg.norm(goals - np.array([drone.position for drone in drones]), axis=1) > epsilon) and i < NUMBER_OF_TIMESTEPS:
    if i % 10 == 0:
        print("Iteration {}".format(i))
    path = []
    control = []
    for idx in range(num_robot):
        goal = goals[idx]

        # compute velocity using nmpc
        start = time.time()
        vel, velocity_profile = drones[idx].computeVelocity(goal, drones, obstacles)
        ts.append(time.time() - start)
        drones[idx].updateState(vel, TIMESTEP)
        path.extend(drones[idx].position)
        control.extend(drones[idx].control)
    i += 1
    robot_state_history.append(np.array(path))
    robot_control_history.append(np.array(control))


print('Iteration {}, Max iterarion: {}'.format(i, NUMBER_OF_TIMESTEPS))
ts = np.array(ts)
print("Average time: {:.6}s".format(ts.mean()))
print("Max time: {:.6}s".format(ts.max()))
print("Min time: {:.6}s".format(ts.min()))


robot_state_history = np.array(robot_state_history)
robot_control_history = np.array(robot_control_history)
plot_robot_and_obstacles(robot_state_history, obstacles, ROBOT_RADIUS,OBS_RADIUS, filename)

# Plot velocity profiles
plt.figure()
plt.plot(robot_control_history[:,0], label="ux")
plt.plot(robot_control_history[:,1], label="uy")
plt.legend()
plt.show()