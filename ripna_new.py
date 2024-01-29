import os
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from numpy import e  # If 'e' is used in the code
from hyperparameters import (
    spread, noise, radius, delt, pert, b, sensor_range, n, maxv, attractive_gain, 
    repulsive_gain, collision_distance, clipping_power, seed, priority_type
)
import dubins


# Global data array initialization
global_data_array = np.zeros((7, 13))

# Set the random seed for reproducibility
np.random.seed(seed)
print("Random seed set to:", seed)


# Function to clear all the output files from previous runs.

def clear_previous_output_files():

    """
    Deletes output files from previous runs if they exist.
    This includes 'time.csv', 'priority.csv', and 'avg.csv'.
    """

    for file_name in ['time.csv', 'priority.csv', 'avg.csv']:
        if os.path.isfile(file_name):
            os.remove(file_name)

# Clear output files at the start of the script
clear_previous_output_files()


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

    return start_positions, goal_positions



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

    # (Optional) Save the plot if needed
    #plt.savefig('/Users/jaskiratsingh/IISER-Bhopal/Comparison/PANDA/plots')



# Generator function to create initial setup for UAVs
def initialize_uav_properties(number_of_uavs, radius):
    """
    Generates initial properties for UAVs including position, velocity, and other parameters.

    Parameters:
    - number_of_uavs (int): The number of UAVs.

    Returns:
    - A tuple containing arrays for initial velocities, positions, goals, and other UAV attributes.
    """
    # Initialize arrays for velocity, position, and other attributes
    velocity_u = np.zeros((number_of_uavs, 2)) # if n=8 then 8 rows and 2 columns (x,y)
    velocity_v = np.zeros((number_of_uavs, 2))
    start_positions, goal_positions = initialize_uav_positions_and_goals(number_of_uavs)  # Initialize positions
    acceleration = np.zeros((number_of_uavs, 2))
    completed_status = np.zeros(number_of_uavs)
    clipping_status = np.zeros(number_of_uavs)
    adjusted_max_velocity = maxv * np.ones(number_of_uavs) # adjusted_max_velocity = vmax

    # Initialize velocity_v with non-zero values
    for i in range(number_of_uavs):
        # Example vector pointing from start to goal, normalized and scaled by maxv
        direction = np.array(goal_positions[i]) - np.array(start_positions[i])
        if np.linalg.norm(direction) > 0:  # Avoid division by zero
            normalized_direction = direction / np.linalg.norm(direction)
            velocity_v[i] = normalized_direction * maxv  # Scale by maxv or any other desired value

    return velocity_u, velocity_v, start_positions, goal_positions, acceleration, completed_status, clipping_status, adjusted_max_velocity

"""
def set_uav_priorities(number_of_uavs, priority_type):
    """
    #Sets the priorities for each UAV based on the specified priority type.

    #Parameters:
    #- number_of_uavs (int): The number of UAVs.
    #- priority_type (str): The type of priority to set ('Gaussian', 'Uniform', or 'Constant').

    #Returns:
    #- priority (np.array): Array of priorities for each UAV.
"""
    if priority_type == "Gaussian":
        priority = np.random.normal(3, 1, number_of_uavs)
    elif priority_type == "Uniform":
        priority = np.random.uniform(1, 6, number_of_uavs)
    else:
        priority = 3 * np.ones(number_of_uavs)

    # Save priorities to a file
    with open("priority.csv", 'a') as priority_file:
        priority_file.write(", ".join([str(x) for x in priority]) + '\n')

    return priority
"""


# Function to check if a UAV has reached its goal
def reached_goal(uav_index, start_positions, goal_positions, radius):
    """
    Checks if a UAV has reached its goal. (Continuation)

    Parameters:
    - uav_index (int): The index of the UAV.
    - positions (np.array): Array of current positions of UAVs.
    - goals (np.array): Array of goal positions of UAVs.
    - radius (float): The radius within which a goal is considered reached.

    Returns:
    - bool: True if the UAV has reached its goal, False otherwise.
    """

    start_positions = np.array(start_positions[uav_index])
    goal_positions = np.array(goal_positions[uav_index])

    if np.linalg.norm(start_positions - goal_positions) <= radius / 100:
        return True
    else:
        return False



def calculate_time_to_go(uav_i, uav_j, velocity_v):
    """
    Calculate the time-to-go (tgo) for the closest approach between two UAVs.

    Parameters:
    uav_i (int): The ID of the first UAV.
    uav_j (int): The ID of the second UAV.

    Returns:
    float: The time-to-go (tgo) for the closest approach.
    """
    relative_velocity = velocity_v[uav_i] - velocity_v[uav_j]
    time_to_go = -np.dot(np.array(start_positions[uav_i]) - np.array(start_positions[uav_j]), relative_velocity) / (np.dot(relative_velocity, relative_velocity) + 0.000001)

    return time_to_go


def calculate_zero_effort_miss(uav_i, uav_j, start_positions, velocity_v):
    """
    Calculate the Zero Effort Miss (ZEM) distance between two UAVs.

    Parameters:
    uav1_id (int): The ID of the first UAV.
    uav2_id (int): The ID of the second UAV.

    Returns:
    float: The Zero Effort Miss (ZEM) distance.
    """
    time_to_go = calculate_time_to_go(uav_i, uav_j, velocity_v)
    zem_squared = np.dot(start_positions[uav_i] + velocity_v[uav_i] * time_to_go - start_positions[uav_j] - velocity_v[uav_j] * time_to_go,
                                    start_positions[uav_i] + velocity_v[uav_i] * time_to_go - start_positions[uav_j] - velocity_v[uav_j] * time_to_go)
    return np.sqrt(zem_squared)

"""
def collision(uav_i, uav_j, start_positions, collision_distance, sensor_range, velocity_v):
    """
    #Checks if there is an imminent collision between two UAVs. (here pos -> start_positions from panda)

    #Parameters:
    #- uav_i, uav_j (int): Indices of the two UAVs being checked.
    #- positions, velocities (np.array): Arrays of positions and velocities of UAVs.
    #- collision_distance (float): The distance threshold for a collision.
    #- sensor_range (float): The sensor range of the UAVs.

    #Returns:
    #- bool: True if a collision is imminent, False otherwise.
"""
    time_to_go = calculate_time_to_go(uav_i, uav_j, start_positions)

    if time_to_go < 0:
        return False
    else:

        zem_squared = calculate_zero_effort_miss(uav1_id, uav2_id)

        actual_distance_btw_2_uavs = np.linalg.norm(np.array(pos[uav1_id]) - np.array(pos[uav2_id]))

        if np.sqrt(zem_squared) < collision_distance and actual_distance_btw_2_uavs < sensor_range:
            return True
        else:
            return False
"""

# Function to check for potential collisions between UAVs
def collision(uav_i, uav_j, start_positions, velocity_v, velocity_u, collision_distance, sensor_range):
    """
    Checks if there is an imminent collision between two UAVs.

    Parameters:
    - uav_i, uav_j (int): Indices of the two UAVs being checked.
    - positions, velocities (np.array): Arrays of positions and velocities of UAVs.
    - collision_distance (float): The distance threshold for a collision.
    - sensor_range (float): The sensor range of the UAVs.

    Returns:
    - bool: True if a collision is imminent, False otherwise.
    """
    relative_velocity = velocity_v[uav_i] - velocity_v[uav_j]
    t = -np.dot(np.array(start_positions[uav_i]) - np.array(start_positions[uav_j]), relative_velocity) / (np.dot(relative_velocity, relative_velocity) + 0.000001)

    if t < 0:
        return False
    else:
        # projected_distance -> s_quared and dist -> actual distance
        projected_distance = np.dot(start_positions[uav_i] + velocity_v[uav_i] * t - start_positions[uav_j] - velocity_v[uav_j] * t,
                                    start_positions[uav_i] + velocity_v[uav_i] * t - start_positions[uav_j] - velocity_v[uav_j] * t)
        actual_distance_btw_2_uavs = np.linalg.norm(np.array(start_positions[uav_i]) - np.array(start_positions[uav_j]))

        if np.sqrt(projected_distance) < collision_distance and actual_distance_btw_2_uavs < sensor_range:
            return True
        else:
            return False



def calculate_turn_radius(ZEM, Rmin, lambda_param, Rdes):
    """
    Calculate the turn radius for the UAVs based on the desired separation and other parameters. (desired separation is sensor range)

    Parameters:
    - ZEM (float): Predicted minimum separation.
    - Rdes (float): Desired separation.
    - Rmin (float): Minimum radius of turn.
    - lambda_param (float): Tuning parameter.

    Returns:
    - float: Calculated turn radius.
    """
    R = Rmin * np.exp(lambda_param * ZEM / Rdes)

    return R

"""
# Function to clip the velocity of UAVs
def clip_velocity(number_of_uavs):
    global velocity_v, clipping_status
    """
    #Adjusts the velocities of UAVs to prevent high-speed collisions and ensure safe operation.

    #This function scales down the velocity of each UAV if it exceeds a dynamically adjusted maximum velocity.
    #This maximum velocity is determined based on the UAV's priority and the collision risk with other UAVs.

    #Parameters:
    #- number_of_uavs (int): The total number of UAVs in the simulation.

    #Modifies:
    #- velocity_v (global numpy.ndarray): The velocities of each UAV.
    #- clipping_status (global numpy.ndarray): Indicates whether a UAV's velocity needs to be clipped.
    #- adjusted_max_velocity (global numpy.ndarray): The adjusted maximum velocities for each UAV.
"""
    #print("----------------")
    for i in range(number_of_uavs):
        
        if clipping_status[i]:
            adjusted_max_velocity[i] = maxv*(priority[i]/max(in_collision[i]))**clipping_power
        else:
            adjusted_max_velocity[i] = maxv
        #vmax[i]=maxv
        if np.linalg.norm(velocity_v[i])>adjusted_max_velocity[i]:
            velocity_v[i] = adjusted_max_velocity[i]*velocity_v[i]/np.linalg.norm(velocity_v[i])



def perturb_velocity(velocity_v):
    """
    #Perturbs the velocities of UAVs by a slight random amount.

    #Parameters:
    #- velocities (np.array): Array of current velocities of UAVs.

    #Modifies the velocities array in place.
"""
    for i, vel in enumerate(velocity_v):
        x = vel[0]
        y = vel[0]
        delta_theta = np.random.normal(0, np.pi / 2 ** 10.5)
        theta = np.arctan2(vel[1], vel[0])
        theta_perturbed = theta + delta_theta

        # Calculate the perturbed vector components using the perturbed angle
        x_perturbed = np.cos(theta_perturbed) * np.sqrt(x**2 + y**2)
        y_perturbed = np.sin(theta_perturbed) * np.sqrt(x**2 + y**2)

        #magnitude = np.sqrt(vel[0] ** 2 + vel[1] ** 2)

        velocity_v[i] = np.array([x_perturbed, y_perturbed])

"""

def rotate_vector_clockwise_90(vector):
    """
    Rotates a 2D vector by 90 degrees in the clockwise direction.

    Parameters:
    - vector (np.array): The vector to be rotated.

    Returns:
    - np.array: The rotated vector.
    """
    x, y = vector[0], vector[1]
    return np.array([y, -x])  # 90 degrees clockwise rotation


def calculate_dubins_path(start, goal, turning_radius):
    """
    Calculate the Dubins path from start to goal for a UAV.

    Parameters:
    - start: The starting pose of the UAV (x, y, theta).
    - goal: The goal pose of the UAV (x, y, theta).
    - turning_radius: The minimum turning radius of the UAV.

    Returns:
    - A list of poses representing the Dubins path.
    """
    # Dubins path configuration
    dubins_path = dubins.shortest_path(start, goal, turning_radius)
    # Sample the path at intervals of 'delt'
    configurations, _ = dubins_path.sample_many(delt)
    
    return configurations



if __name__ == "__main__":

    
    if 'number_of_uavs' not in globals():
        number_of_uavs = 8  # Default number of UAVs
    if 'radius' not in globals():
        radius = 50  # Default radius
    if 'delt' not in globals():
        delt = 0.01  # Default time delta
    if 'maxv' not in globals():
        maxv = 1.0  # Default maximum velocity

    min_dist = np.inf*np.ones((number_of_uavs, number_of_uavs))
    avg_dist = np.zeros(number_of_uavs)

    mission_completion = [0]*number_of_uavs

    # Initialize UAV positions and goals
    start_positions, goal_positions = initialize_uav_positions_and_goals(number_of_uavs)

    # Plot initial positions and goals (optional)
    plot_initial_positions_and_goals(start_positions, goal_positions, number_of_uavs, radius)

    velocity_u, velocity_v, start_positions, goal_positions, acceleration, completed_status, clipping_status, adjusted_max_velocity = initialize_uav_properties(number_of_uavs, radius)
    #priority_type = "Gaussian"
    #priority = set_uav_priorities(number_of_uavs, priority_type)  # or 'Uniform', 'Constant'
    #collision_priorities = priority
    #in_collision = [[i] for i in priority]
    path_indices = [0] * number_of_uavs

    # File initialization for output
    file = open('out_ripna.csv', 'w')
    velocity_file = open('vel_ripna.csv', 'w')
    time_file = open('time_ripna.csv', 'a')
    acc_file = open('acc_ripna.csv', 'w')

    # Write headers for the files
    #header = ','.join([f"{i}x,{i}y" for i in range(number_of_uavs)])
    header = ','.join(["{}x,{}y".format(i, i) for i in range(number_of_uavs)])
    file.write(header + "\n")
    velocity_file.write(header + "\n")
    acc_file.write(header + "\n")


    r = 0
    #Main Simulation loop
    while not np.array_equal(completed_status, np.ones(number_of_uavs)):

        r += 1
        clipping_status = np.zeros(number_of_uavs)
        acceleration = np.zeros((number_of_uavs,2))
        for i in range(number_of_uavs):
            if reached_goal(i, start_positions, goal_positions, radius):
                if completed_status[i] != 1:
                    completed_status[i] = 1
                    acceleration[i] = 0
                    velocity_v[i] = 0
                    mission_completion[i]=r


            else:
                # calculates the attractive force 
                #dist_of_uav_to_goal = np.linalg.norm(np.array(goal_positions[i]) - np.array(start_positions[i]))
                #attractive_force = 2*(1-e**(-2*dist_of_uav_to_goal**2))*(np.array(goal_positions[i])-np.array(start_positions[i]))/dist_of_uav_to_goal
                #attractive_force = attractive_gain*np.array(goal[i]-pos[i])/dist_to_goal
                #acceleration[i] = np.array(attractive_force)
                #print("a",i,a[i])

                # Calculate Repulsive Force
                colliding=False
                for j in range(number_of_uavs):
                    dist_of_uav1_from_uav2 = np.linalg.norm(np.array(start_positions[j]) - np.array(start_positions[i])) # Calculating distance of UAV j from UAV i
                    if dist_of_uav1_from_uav2 < min_dist[i][j]:
                        min_dist[i][j] = dist_of_uav1_from_uav2
                    if j != i: # checking if UAV is not checking collision with itself
                        if collision(i, j, start_positions, velocity_v, velocity_u, collision_distance, sensor_range):
                            colliding=True
                            #print("collision",i,j)
                            # Calculate lateral acceleration direction by rotating velocity vector by 90 degrees clockwise
                            #print(velocity_v[i])
                            direction_for_acceleration = rotate_vector_clockwise_90(velocity_v[i])
                            # Calculate the magnitude of acceleration based on velocity magnitude and turn radius
                            V_magnitude = np.linalg.norm(velocity_v[i])
                            ZEM = calculate_zero_effort_miss(i, j, start_positions, velocity_v)
                            R = calculate_turn_radius(ZEM, Rmin=20, lambda_param=0.1, Rdes=1)  # lambda_param is a tuning parameter
                            acceleration_magnitude = V_magnitude ** 2 / R
                            # Apply acceleration in the direction obtained from the rotated velocity vector
                            norm = np.linalg.norm(direction_for_acceleration)
                            if norm > 0:
                                acceleration[i] = direction_for_acceleration / norm * acceleration_magnitude
                            else:
                                acceleration[i] = np.zeros_like(direction_for_acceleration)

                            #print(acceleration[i])

                            


                if colliding: #The clip array is used to keep track of which UAVs need to adjust their movement to avoid collisions. By setting clip[i] = 1, the code is marking UAV i for such an adjustment.
                    clipping_status[i]=1

                #print(velocity_v + acceleration*delt)


                """if not colliding:
                    # Calculate the UAV's current orientation based on its velocity vector
                    theta = np.arctan2(velocity_v[i][1], velocity_v[i][0])
                    # Define the start and goal configurations for the Dubins path calculation
                    start_config = (start_positions[i][0], start_positions[i][1], theta)
                    # Assuming goal orientation of 0 radians. This can be adjusted as needed.
                    goal_config = (goal_positions[i][0], goal_positions[i][1], 0)

                    # Calculate the Dubins path using the minimum turning radius
                    dubins_path = calculate_dubins_path(start_config, goal_config, radius)  # 'radius' is your UAV turning radius

                    # Use the first segment of the Dubins path to set the UAV's velocity
                    if dubins_path:
                        next_pose = dubins_path[1]  # Taking the next pose from the sampled Dubins path
                        direction = np.array([next_pose[0] - start_positions[i][0], next_pose[1] - start_positions[i][1]])
                        # Normalize the direction and scale by max velocity
                        velocity_v[i] = direction / np.linalg.norm(direction) * maxv"""

                if not colliding:
                    if path_indices[i] == 0:  # Path needs to be (re)calculated
                        theta = np.arctan2(velocity_v[i][1], velocity_v[i][0])
                        start_config = (start_positions[i][0], start_positions[i][1], theta)
                        goal_theta = np.arctan2(goal_positions[i][1] - start_positions[i][1], goal_positions[i][0] - start_positions[i][0])
                        goal_config = (goal_positions[i][0], goal_positions[i][1], goal_theta)
                        dubins_path = calculate_dubins_path(start_config, goal_config, radius)
                        if dubins_path and len(dubins_path) > 1:
                            path_indices[i] = 1  # Start following the path

                    if 0 < path_indices[i] < len(dubins_path):
                        next_pose = dubins_path[path_indices[i]]
                        direction = np.array([next_pose[0] - start_positions[i][0], next_pose[1] - start_positions[i][1]])
                        if np.linalg.norm(direction) > 0:
                            velocity_v[i] = direction / np.linalg.norm(direction) * maxv
                            #print(velocity_v[i])
                        path_indices[i] += 1


        #if pert:
            #perturb_velocity(velocity_v)
        velocity_v = velocity_v + acceleration*delt
        #print(velocity_v)
        #clip_velocity(number_of_uavs)
        start_positions = start_positions + velocity_v*delt

        file.write(",".join([str(x) for x in start_positions.flatten()])+"\n")
        velocity_file.write(", ".join([str(x) for x in velocity_v.flatten()])+"\n")
        acc_file.write(", ".join([str(x) for x in acceleration.flatten()])+"\n")
    print(completed_status)
    #print(check)
    time_file.write(", ".join([str(x) for x in mission_completion])+'\n')
    time_file.close()

    file.close()
    velocity_file.close() 


    acc_file.close()

    for i in range(n):
        avg_dist[i] = np.mean(min_dist[i])
    
    avg_file = open('avg.csv','a')
    avg_file.write(",".join([str(x) for x in avg_dist])+"\n")
    # end = time.time()
    # excec_time = end-start
    # exec_file.write(f'{i},{excec_time}')
    #print("n:",number_of_uavs, "priority: ",priority)
    avg_file.close()



"""
number_of_uavs: Number of UAVs needs to be in the multiple of with minimum of 4 UAVs
adjusted_max_velocity = vmax
"""




