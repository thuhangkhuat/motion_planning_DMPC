import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np


def plot_robot_and_obstacles(robot, obstacles, robot_radius,obs_radius, filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 35), ylim=(0, 12))
    ax.set_aspect('equal')
    ax.grid()

    num_robot = int(robot.shape[1]/2)

    lines = []
    for i in range(num_robot):
        line, = ax.plot([], [], '--k')
        lines.append(line)
    
    robot_patches = []
    for i in range(num_robot):
        robot_patch = Circle((robot[2*i, 0], robot[2*i+1, 0]),
                            robot_radius, facecolor='green', edgecolor='black')
        robot_patches.append(robot_patch)
    
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[0]):
        obstacle = Circle((0, 0), obs_radius,
                          facecolor='blue', edgecolor='black')
        obstacle_list.append(obstacle)

    def init():
        for i in range(num_robot):
            ax.add_patch(robot_patches[i])
            lines[i].set_data([], [])

        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        return robot_patches + lines + obstacle_list

    def animate(i):        
        for j in range(num_robot):
            robot_patches[j].center = (robot[i,2*j], robot[i,2*j+1])
            lines[j].set_data(robot[:i,2*j], robot[:i,2*j+1])

        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[j,0], obstacles[j,1])
        return robot_patches + lines + obstacle_list

    init()

    for i in range(robot.shape[0]):
        animate(i)
        plt.pause(0.05)

    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, robot.shape[0]), interval=200,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)


def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    if is_obstacle:
        circle = plt.Circle((x, y), radius, color='aqua', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)
    else:
        circle = plt.Circle((x, y), radius, color='green', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')

    plt.gcf().gca().add_artist(circle)
