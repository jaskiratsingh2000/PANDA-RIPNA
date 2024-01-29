import numpy as np
import matplotlib.pyplot as plt
import dubins
from hyperparameters import (
    spread
)

# Constants and Parameters
NUMBER_OF_UAVS = 8
RADIUS = 2
TURNING_RADIUS = 0.1
DELT = 0.01
MAXV = 1.0
LATERAL_ACCELERATION = 0.5
COLLISION_DISTANCE = 5
SENSOR_RANGE = 100
EPSILON = 0.1
K = 0.5

np.random.seed(42)  # For reproducibility

# Function to initialize the starting and goal positions of UAVs
def initialize_uav_positions_and_goals(number_of_uavs):
    """
    Initializes the starting and goal positions for a given number of UAVs.
    UAVs on the left side of the innermost diameter have their goals on the right side of the outermost diameter, and vice versa.
    
    Parameters:
    - number_of_uavs (int): The number of UAVs to initialize positions for.

    Returns:
    - start_positions (list): List of starting positions for each UAV.
    - goal_positions (list): List of goal positions for each UAV.
    """
    start_positions = []
    goal_positions = []

    for i in range(1, number_of_uavs // 4 + 1):
        position1 = (np.cos(spread * (i / (number_of_uavs // 4))), np.sin(spread * (i / (number_of_uavs // 4))))
        position2 = (-position1[0], position1[1])
        position3 = (-position1[0], -position1[1])
        position4 = (position1[0], -position1[1])

        start_positions.extend([position1, position2, position3, position4])
        goal_positions.extend([position2, position1, position4, position3])

    return np.array(start_positions), np.array(goal_positions)



def plot_initial_positions_and_goals(start_positions, goal_positions, number_of_uavs, radius):
    """
    Plots the initial positions and goal positions of the UAVs.
    
    Parameters:
    - start_positions (list): List of starting positions for each UAV.
    - goal_positions (list): List of goal positions for each UAV.
    - number_of_uavs (int): The number of UAVs.
    - radius (float): The radius of the circle for initial positioning.
    """
    # Generate colors for each UAV
    cmap = plt.get_cmap('hsv')
    colors = [cmap(i) for i in np.linspace(0, 1, len(start_positions))]
    #colors = [cmap(i) for i in np.linspace(0, 1, number_of_uavs + 2)][2:]

    # Create a plot
    fig, ax = plt.subplots(figsize=(20, 10))

    # Scatter plot for starting positions
    plt.scatter(radius * np.array(start_positions)[:, 0], 
                radius * np.array(start_positions)[:, 1], 
                c=colors, s=200)

    # Scatter plot for goal positions
    plt.scatter(1.2 * radius * np.array(goal_positions)[:, 0], 
                1.2 * radius * np.array(goal_positions)[:, 1], 
                c=colors, marker="s", s=200, alpha=0.5)

    # Set plot limits and labels
    plt.xlim(-1.2 * radius, 1.2 * radius)
    plt.ylim(-radius / 2, radius / 2)
    plt.xlabel('x', fontsize=24)
    plt.ylabel('y', fontsize=24)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)

    # Draw circles to indicate boundaries
    plt.gca().add_patch(plt.Circle((0, 0), radius, fill=False))
    plt.gca().add_patch(plt.Circle((0, 0), 1.2 * radius, fill=False))

def collision(uav_index, start_positions, velocity_v):
    collision_uavs = []
    for j in range(NUMBER_OF_UAVS):
        if j != uav_index:
            distance = np.linalg.norm(start_positions[uav_index] - start_positions[j])
            if distance < COLLISION_DISTANCE:
                collision_uavs.append(j)
    return collision_uavs

def calculate_dubins_path(start, goal, turning_radius):
    path = dubins.shortest_path((start[0], start[1], 0), (goal[0], goal[1], 0), turning_radius)
    configurations, _ = path.sample_many(DELT)
    return configurations

def adjust_heading(start_position, goal_position, current_heading):
    goal_heading = np.arctan2(goal_position[1] - start_position[1], goal_position[0] - start_position[0])
    heading_error = goal_heading - current_heading
    heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
    new_heading = current_heading + K * heading_error * DELT
    return new_heading

def apply_lateral_acceleration(uav_index, start_positions, velocity_v, collision_uavs):
    for other_uav in collision_uavs:
        direction_to_other_uav = start_positions[other_uav] - start_positions[uav_index]
        perpendicular_direction = np.array([-direction_to_other_uav[1], direction_to_other_uav[0]])
        perpendicular_direction_normalized = perpendicular_direction / np.linalg.norm(perpendicular_direction)
        velocity_v[uav_index] += perpendicular_direction_normalized * LATERAL_ACCELERATION * DELT
        if np.linalg.norm(velocity_v[uav_index]) > MAXV:
            velocity_v[uav_index] = velocity_v[uav_index] / np.linalg.norm(velocity_v[uav_index]) * MAXV

def update_uav_trajectory(uav_index, start_positions, goal_positions, velocity_v):
    current_pos = start_positions[uav_index]
    goal_pos = goal_positions[uav_index]
    current_heading = np.arctan2(velocity_v[uav_index][1], velocity_v[uav_index][0])

    collision_uavs = collision(uav_index, start_positions, velocity_v)
    if collision_uavs:
        apply_lateral_acceleration(uav_index, start_positions, velocity_v, collision_uavs)
    elif np.linalg.norm(goal_pos - current_pos) <= TURNING_RADIUS or np.abs(adjust_heading(current_pos, goal_pos, current_heading) - current_heading) < EPSILON:
        new_heading = adjust_heading(current_pos, goal_pos, current_heading)
        velocity_v[uav_index] = np.array([np.cos(new_heading), np.sin(new_heading)]) * MAXV
    else:
        dubins_path = calculate_dubins_path(current_pos, goal_pos, TURNING_RADIUS)
        if len(dubins_path) > 1:
            next_point = dubins_path[1]
            direction = next_point[:2] - current_pos
            if np.linalg.norm(direction) > 0:
                velocity_v[uav_index] = (direction / np.linalg.norm(direction)) * MAXV

    start_positions[uav_index] += velocity_v[uav_index] * DELT

def main():
    # Initialize UAV positions and goals
    start_positions, goal_positions = initialize_uav_positions_and_goals(NUMBER_OF_UAVS)
    velocity_v = np.zeros((NUMBER_OF_UAVS, 2))
    completed_status = np.zeros(NUMBER_OF_UAVS)

    # Plot initial positions and goals (optional)
    plot_initial_positions_and_goals(start_positions, goal_positions, NUMBER_OF_UAVS, RADIUS)

    while not np.all(completed_status):
        for i in range(NUMBER_OF_UAVS):
            if not completed_status[i]:
                update_uav_trajectory(i, start_positions, goal_positions, velocity_v)
                if np.linalg.norm(start_positions[i] - goal_positions[i]) <= RADIUS / 100:
                    completed_status[i] = 1
                    velocity_v[i] = np.zeros(2)

        plt.clf()
        for i in range(NUMBER_OF_UAVS):
            plt.plot([start_positions[i][0]], [start_positions[i][1]], 'bo')
            plt.plot([goal_positions[i][0]], [goal_positions[i][1]], 'rx')
        plt.xlim(-1.2 * RADIUS, 1.2 * RADIUS)
        plt.ylim(-1.2 * RADIUS, 1.2 * RADIUS)
        plt.pause(0.01)

    plt.show()

if __name__ == "__main__":
    main()
