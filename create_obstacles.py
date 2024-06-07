import numpy as np

def create_obstacles(sim_time, num_timesteps):
    # Obstacle 1
    p0 = np.array([7., 3.])
    obstacles = p0
    # Obstacle 2
    p0 = np.array([7., 6.])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 3
    p0 = np.array([7., 8.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 4
    p0 = np.array([9., 1.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 5
    p0 = np.array([12.0, 4.0])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 6
    p0 = np.array([12.0, 7.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 7
    p0 = np.array([15., 1.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 8    
    p0 = np.array([15., 6.])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 9
    p0 = np.array([17., 8.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 10
    p0 = np.array([19., 3.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 11
    p0 = np.array([19., 6.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 12
    p0 = np.array([22., 2.])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 13
    p0 = np.array([23., 5.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 14
    p0 = np.array([24., 8.])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 15
    p0 = np.array([26., 4.])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 16
    p0 = np.array([29, 2.5])
    obstacles = np.vstack((obstacles, p0))
    # Obstacle 17
    p0 = np.array([29., 7.5])
    obstacles = np.vstack((obstacles, p0))
    # obstacles = []
    return obstacles