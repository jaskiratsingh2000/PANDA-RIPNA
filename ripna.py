import numpy as np
#import dubins


# Constants and Initialization
n = 4
vmax = 5
delt = 0.001
rdes = 2
Rmin = 1  # Minimum turning radius
lambda_param = 0.5

# Initialize UAV states
#pos = np.random.rand(n, 2) * 10  # Random positions
#goal = np.random.rand(n, 2) * 10  # Random goals
#vel = np.zeros((n, 2))  # Initial velocities
#orientations = np.random.rand(n) * 2 * np.pi  # Random orientations

"""
n=4
vmax=5
delt=0.001
rdes=2

pos = []
vel = []
goal = []
counter=0

uavs = []
"""
#Assuming they have same velocities


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
    plt.savefig('/Users/jaskiratsingh/IISER-Bhopal/Comparison/PANDA/plots')


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
    #velocity = np.zeros((number_of_uavs, 2)) # if n=8 then 8 rows and 2 columns (x,y)
    velocity = vmax
    start_positions, goal_positions = initialize_uav_positions_and_goals(number_of_uavs)  # Initialize positions
    acceleration = np.zeros((number_of_uavs, 2))
    completed_status = np.zeros(number_of_uavs)
    clipping_status = np.zeros(number_of_uavs)
    #adjusted_max_velocity = maxv * np.ones(number_of_uavs) # adjusted_max_velocity = vmax

    return velocity, start_positions, goal_positions, acceleration, completed_status, clipping_status #adjusted_max_velocity


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


def calculate_time_to_go(uav1_id, uav2_id):
    """
    Calculate the time-to-go (tgo) for the closest approach between two UAVs.

    Parameters:
    uav1_id (int): The ID of the first UAV.
    uav2_id (int): The ID of the second UAV.

    Returns:
    float: The time-to-go (tgo) for the closest approach.
    """
    relative_position = pos[uav1_id] - pos[uav2_id]
    #relative_velocity = vel[uav1_id] - vel[uav2_id]
    velocity = vmax
    numerator = np.dot(relative_position, velocity)
    denominator = np.dot(relative_velocity, velocity) + 0.000001

    return -numerator / denominator


def calculate_zero_effort_miss(uav1_id, uav2_id, pos):
    """
    Calculate the Zero Effort Miss (ZEM) distance between two UAVs.

    Parameters:
    uav1_id (int): The ID of the first UAV.
    uav2_id (int): The ID of the second UAV.

    Returns:
    float: The Zero Effort Miss (ZEM) distance.
    """
    time_to_go = calculate_time_to_go(uav1_id, uav2_id)
    relative_position = pos[uav1_id] - pos[uav2_id]
    relative_velocity = vel[uav1_id] - vel[uav2_id]
    zem_squared = np.dot(relative_position, relative_position) + \
                  2 * np.dot(relative_position, relative_velocity) * vmax * time_to_go + \
                  np.dot(relative_velocity, relative_velocity) * (vmax * time_to_go)**2
    return np.sqrt(zem_squared)


def collision(uav1_id, uav2_id, pos, collision_distance, sensor_range):
    """
    Checks if there is an imminent collision between two UAVs. (here pos -> start_positions from panda)

    Parameters:
    - uav_i, uav_j (int): Indices of the two UAVs being checked.
    - positions, velocities (np.array): Arrays of positions and velocities of UAVs.
    - collision_distance (float): The distance threshold for a collision.
    - sensor_range (float): The sensor range of the UAVs.

    Returns:
    - bool: True if a collision is imminent, False otherwise.
    """
    time_to_go = calculate_time_to_go(uav1_id, uav2_id, pos)

    if time_to_go < 0:
        return False
    else:

        zem_squared = calculate_zero_effort_miss(uav1_id, uav2_id)

        actual_distance_btw_2_uavs = np.linalg.norm(np.array(pos[uav1_id]) - np.array(pos[uav2_id]))

        if np.sqrt(zem_squared) < collision_distance and actual_distance_btw_2_uavs < sensor_range:
            return True
        else:
            return False


def calculate_los_rate(uav1_id,uav2_id, pos, velocity):
    """
    Calculate the Line of Sight (LOS) rate based on positions and velocities of two UAVs.

    Parameters:
    - uav1_position, uav2_position (np.array): Position vectors of UAV1 and UAV2.
    - uav1_velocity, uav2_velocity (np.array): Velocity vectors of UAV1 and UAV2.

    Returns:
    - float: The LOS rate.
    """
    r = np.linalg.norm(pos[uav2_id] - pos[uav1_id])  # LOS separation
    #theta1 = np.arcsin(np.cross(velocity, pos[uav1_id]- pos[uav2_id]) / (np.linalg.norm(velocity) * r))
    #theta2 = np.arcsin(np.cross(velocity, pos[uav2_id] - pos[uav1_id]) / (np.linalg.norm(velocity) * r))
    #theta1 = np.arctan2(relative_position1[1], relative_position1[0]) - np.arctan2(uav1_velocity[1], uav1_velocity[0])
    #theta2 = np.arctan2(relative_position2[1], relative_position2[0]) - np.arctan2(uav2_velocity[1], uav2_velocity[0])

    los_rate = (np.sin(theta2) - np.sin(theta1)) * np.linalg.norm(velocity) / r  # Assuming equal velocity for both UAVs

    return loss_rate

def calculate_turn_radius(ZEM, Rdes, Rmin, lambda_param):
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



def determine_turn_direction(uav_id, other_uav_id, positions, velocities):
    """
    Determines the direction to turn to avoid a collision based on the relative position of another UAV.

    Parameters:
    - uav_id (int): Identifier for the UAV that needs to turn.
    - other_uav_id (int): Identifier for the other UAV.
    - positions (np.array): Array of UAV positions.
    - velocities (np.array): Array of UAV velocities.

    Returns:
    - str: 'left' or 'right', indicating the direction to turn.
    """
    # Calculate the relative position vector of the other UAV from the perspective of the current UAV
    relative_position = positions[other_uav_id] - positions[uav_id]

    # Calculate the heading vector of the current UAV
    heading_vector = velocities[uav_id]
    heading_angle = np.arctan2(heading_vector[1], heading_vector[0])

    # Calculate the angle of the relative position vector
    relative_angle = np.arctan2(relative_position[1], relative_position[0])

    # Determine the turn direction
    angle_difference = relative_angle - heading_angle
    if angle_difference < 0:
        angle_difference += 2 * np.pi  # Ensure the angle is within 0 to 2π

    if angle_difference > np.pi:
        return 'left'
    else:
        return 'right'

# Example usage
#turn_direction = determine_turn_direction(uav1_id, uav2_id, start_positions, velocities)



def apply_lateral_acceleration(uav_id, start_positions, velocity, lateral_acc, delt):
    """
    Applies lateral acceleration to a UAV to change its direction of motion.

    Parameters:
    - uav_id (int): Identifier of the UAV.
    - start_positions (numpy.ndarray): Array of UAV positions.
    - velocity (float): Velocity of the UAVs.
    - lateral_acc (float): Lateral acceleration to be applied.
    - delt (float): Time step for the simulation.
    """
    # Calculate the direction of the velocity vector
    direction = np.arctan2(start_positions[uav_id][1], start_positions[uav_id][0])

    # Calculate the change in direction due to lateral acceleration
    delta_direction = lateral_acc / velocity * delt

    # Update the direction
    new_direction = direction + delta_direction

    # Update the velocity vector based on the new direction
    velocity_x = velocity * np.cos(new_direction)
    velocity_y = velocity * np.sin(new_direction)
    return np.array([velocity_x, velocity_y])




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

    velocity, start_positions, goal_positions, acceleration, completed_status, clipping_status = initialize_uav_properties(number_of_uavs, radius)
    #priority_type = "Gaussian"
    #priority = set_uav_priorities(number_of_uavs, priority_type)  # or 'Uniform', 'Constant'
    #collision_priorities = priority
    #in_collision = [[i] for i in priority]

    
    """# File initialization for output
    file = open('out.csv', 'w')
    velocity_file = open('vel.csv', 'w')
    time_file = open('time.csv', 'a')
    acc_file = open('acc.csv', 'w')

    # Write headers for the files
    header = ','.join([f"{i}x,{i}y" for i in range(number_of_uavs)])
    file.write(header + "\n")
    velocity_file.write(header + "\n")
    acc_file.write(header + "\n")"""


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
                        if collision(i, j, start_positions, collision_distance, sensor_range):
                            colliding=True
                            #print("collision",i,j)
                            #dist = np.linalg.norm(pos[j]-pos[i])
                            #repulsive_force = (priority[j]/priority[i])*repulsive_gain*(e**(-b*(dist_of_uav1_from_uav2-collision_distance)**2))*(np.array(start_positions[i]) - np.array(start_positions[j]))/dist_of_uav1_from_uav2
                            #repulsive_force = rotate_vector(repulsive_force, np.pi/2) # rotating the force vector
                            #print(i,j,dist,rep,a[i])

                            # Calculate LOS rate
                            los_rate = calculate_los_rate(i, j, start_positions, velocity)

                            # Determine if we need to increase LOS rate based on the logic provided
                            if los_rate < 0:
                                turn_direction_uav1 = determine_turn_direction(i, los_rate, start_positions, vmax)
                                turn_direction_uav2 = determine_turn_direction(j, los_rate, start_positions, vmax)

                                zem = calculate_zero_effort_miss(uav1_id, uav2_id, start_positions)
                                turn_radius = calculate_turn_radius(zem, rdes, Rmin, lambda_param)
                                
                                lateral_acc_uav1 = vmax**2 / turn_radius
                                lateral_acc_uav2 = vmax**2 / turn_radius

                                # Apply the lateral acceleration;
                                velocity[i] = apply_lateral_acceleration(i, vmax, lateral_acc_uav1, turn_direction_uav1, delt)
                                velocity[j] = apply_lateral_acceleration(j, vmax, lateral_acc_uav2, turn_direction_uav2, delt)

                                # Update positions
                                start_positions[i] += velocity[i] * delt
                                start_positions[j] += velocity[j] * delt

                            #acceleration[i] += repulsive_force


                if colliding: #The clip array is used to keep track of which UAVs need to adjust their movement to avoid collisions. By setting clip[i] = 1, the code is marking UAV i for such an adjustment.
                    clipping_status[i]=1
        #if pert:
            #perturb_velocity(velocity_v)
        #velocity_v = velocity_v + acceleration*delt
        #clip_velocity(number_of_uavs)
        #start_positions = start_positions + velocity_v*delt

        #file.write(",".join([str(x) for x in start_positions.flatten()])+"\n")
        #velocity_file.write(", ".join([str(x) for x in velocity_v.flatten()])+"\n")
        #acc_file.write(", ".join([str(x) for x in acceleration.flatten()])+"\n")
    #print(completed_status)
    #print(check)
    #time_file.write(", ".join([str(x) for x in mission_completion])+'\n')
    #time_file.close()

    #file.close()
    #velocity_file.close() 


    #acc_file.close()

    #for i in range(n):
        #avg_dist[i] = np.mean(min_dist[i])
    
    #avg_file = open('avg.csv','a')
    #avg_file.write(",".join([str(x) for x in avg_dist])+"\n")
    # end = time.time()
    # excec_time = end-start
    # exec_file.write(f'{i},{excec_time}')
    #print("n:",number_of_uavs, "priority: ",priority)
    #avg_file.close()



"""
number_of_uavs: Number of UAVs needs to be in the multiple of with minimum of 4 UAVs
adjusted_max_velocity = vmax
"""