import numpy as np
import time

from MPCController import MPCController

class Drone:
    def __init__(self, index:int, position:np.array, control:np.array, radius, control_bound=2.0):
        self.index = index
        self.VREF = 1
        # Drone state and control
        self.position = position
        self.control = control

        # Drone radius
        self.radius = radius

        # Drone control bounds
        self.control_max = [ (1/np.sqrt(2)) * control_bound]*2
        self.control_min = [-(1/np.sqrt(2)) * control_bound]*2
        self.control_bound = control_bound
        self.range_sensing = 5.0
        
        # Store drone path
        self.path = [self.position]
        self.vel_path = [self.control]

        # Initialize controller
        self.controller = MPCController(self.radius, self.control_max, self.control_min)

    def updateState(self, control, timestep):
        """
        Computes the states of drone after applying control signals
        """
        N = int(len(control) / 2)
        lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
        kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

        self.position = np.vstack([np.eye(2)] * int(N)) @ self.position + kron @ control * timestep
        self.control = control
        
        self.path.append(self.position)
        self.vel_path.append(self.control)

    def computeVelocity(self, goal, drones, obstacles):
        observed_obstacles = self.observerObstacles(obstacles)
        observed_drones = self.observerDrones(drones)

        xref = self.compute_xref(goal, self.controller.horizon_length, self.controller.nmpc_timestep)
        # print(np.shape(xref))
        
        # compute velocity using nmpc
        vel, velocity_profile = self.controller.compute_velocity(self.position, observed_drones, observed_obstacles, xref)
        return vel, velocity_profile

    def observerObstacles(self, current_obstacles):
        observed_obstacles = []
        for i in range(np.shape(current_obstacles)[0]):
            obstacle_position = current_obstacles[i,:]
            if np.linalg.norm(self.position-obstacle_position) <= self.range_sensing:
                observed_obstacles.append(obstacle_position)
        return np.array(observed_obstacles)
    
    def observerDrones(self, drones):
        observed_drones = []
        for i in range(len(drones)):
            drone = drones[i]
            if self.index != drone.index:
                if np.linalg.norm(self.position-drone.position) <= self.range_sensing:
                    observed_drones.append(drone)
        return observed_drones

    def compute_xref(self, goal, number_of_steps, timestep):
        dir_vec = (goal - self.position)/np.linalg.norm(goal - self.position)
        path = [self.position]
        for i in range(number_of_steps):
            if np.linalg.norm(goal-path[-1]) <= self.VREF*timestep:
                path.append(goal)
            else:
                path.append(path[-1] + self.VREF*dir_vec*timestep)
        return np.array(path)
   
