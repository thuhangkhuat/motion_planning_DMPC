import numpy as np
from scipy.optimize import minimize, Bounds
import time

class MPCController:
    def __init__(self, radius, upper_bound, lower_bound, horizon_length=10):
        # Boundary
        self.upper_bound = upper_bound*horizon_length
        self.lower_bound = lower_bound*horizon_length

        # nmpc timestep
        self.nmpc_timestep = 0.4

        # Predictive length
        self.horizon_length = horizon_length

        # Control gain
        self.Qc = 1.
        self.kappa = 4.

        # Robot radius
        self.radius = radius


    def compute_velocity(self, position, observed_drones, observed_obstacles, xref):
        """
        Computes control velocity of the copter
        """
        # predict the obstacles' position in future
        drones_predictions = self.predict_drone_positions(observed_drones)

        u0 = np.random.rand(2*self.horizon_length)
        def cost_fn(u): return self.total_cost(
            u, position, drones_predictions, observed_obstacles, xref)

        bounds = Bounds(self.lower_bound, self.upper_bound)
        res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
        velocity = res.x[:2]
        return velocity, res.x
    
    def total_cost(self, u, position, drones_predictions, obstacle_predictions, xref):
        position_next = self.update_state(position, u, self.nmpc_timestep)
        c1 = self.tracking_cost(position_next, xref)
        c2 = self.total_collision_cost(position_next, drones_predictions, obstacle_predictions)
        c3 = self.energy_cost(u)
        total = 0.2*c1 + 0.7*c2 + 0.3*c3
        return total
    
    # @staticmethod
    def update_state(self,x0, u, timestep):
        """
        Computes the states of the system after applying a sequence of control signals u on
        initial state x0
        """
        N = int(len(u) / 2)
        traj = []
        lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
        kron = np.kron(lower_triangular_ones_matrix, np.eye(2))
        # for i in range(self.horizon_length):
        new_state = np.vstack([np.eye(2)] * int(N)) @ x0 + kron @ u * timestep
        new_state = np.array(new_state).reshape(self.horizon_length, 2)
        traj= np.vstack((x0,new_state))
        # print(np.shape(traj))
        return traj
    

    # @staticmethod
    def energy_cost(self,u):
        return np.sum(u**2)
    def tracking_cost(self, traj, traj_ref):
        cost_tra = 0
        for i in range(self.horizon_length):
            # print(np.shape(traj))
            # print(np.shape(traj_ref))
            pos_rel = traj[i,:] - traj_ref[i,:]
            cost_tra += np.sum(pos_rel**2)
        return cost_tra

    def total_collision_cost(self, robot, drones, obstacles):
        total_cost = 0
        for i in range(self.horizon_length):
            # Collision cost with Obstacle
            for j in range(len(obstacles)):
                obstacle = obstacles[j]
                rob = robot[2 * i: 2 * i + 2]
                # obs = obstacle[2 * i: 2 * i + 2]
                total_cost += self.collision_cost(rob, obstacle)
            
            # Collision cost with other Drones
            for j in range(len(drones)):
                obstacle = drones[j]
                rob = robot[2 * i: 2 * i + 2]
                obs = obstacle[2 * i: 2 * i + 2]
                total_cost += self.collision_cost(rob, obs)
        return total_cost

    def collision_cost(self, x0, x1):
        """
        Cost of collision between two robot_state
        """
        d = np.linalg.norm(x0 - x1)
        cost = self.Qc / (1 + np.exp(self.kappa * (d - 2*self.radius)))
        return cost
    
    def predict_drone_positions(self, drones):
        predict_drone_positions = []
        for i in range (len(drones)):
            drone = drones[i]
            drone_position = drone.position
            drone_vel = drone.control
            u = np.vstack([np.eye(2)] * self.horizon_length ) @ drone_vel
            obstacle_prediction = self.update_state(drone_position, u, self.nmpc_timestep)
            predict_drone_positions.append(obstacle_prediction)
        return np.array(predict_drone_positions)
    