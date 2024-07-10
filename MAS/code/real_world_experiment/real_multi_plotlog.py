import matplotlib.pyplot as plt
import cv2
import numpy as np

def convert_sim_to_real_pose(point, matrix):
    point_array = np.array([point['x'], point['y'], 1])
    print(f'sim point {point_array}')
    transformed_point = np.dot(matrix, point_array)
    transformed_point = transformed_point / transformed_point[2]
    print(f'real pose {transformed_point}')
    return transformed_point[:2]


real_points = np.float32([[-0.7633140087127686, 0.47199487686157227],
                          [0.6409847140312195, 0.4631839096546173],
                          [-0.7611741423606873, -0.5574551224708557],
                          [0.7729427814483643, -0.6233859658241272]])
sim_points = np.float32([[0, 0], [10, 0], [0, 10], [10, 10]])

# Calculate the perspective transformation matrix
matrix = cv2.getPerspectiveTransform(real_points, sim_points)


def plot_trajectory(log_file):
    plt.figure(figsize=(10, 10))
    init_poses_agent1, init_poses_agent2 = {}, {}
    with open(log_file, 'r') as f:
        lines = f.readlines()

    current_agent = None
    for line in lines:
        if 'agent 1' in line:
            current_agent = '1'
        elif 'agent 2' in line:
            current_agent = '2'
        elif 'init_pose' in line and current_agent:
            pose = [float(coord) for coord in line.split('[')[1].split(']')[0].split(',')]
            # Applying perspective transformation and y-axis inversion
            transformed_pose = convert_sim_to_real_pose({'x': pose[0], 'y': pose[1]}, matrix)
            if current_agent == '1':
                init_poses_agent1[line] = transformed_pose
            elif current_agent == '2':
                init_poses_agent2[line] = transformed_pose

    # Plotting the initial positions of the agents
    for pose in init_poses_agent1.values():
        plt.plot(pose[0], pose[1],"go")
    for pose in init_poses_agent2.values():
        plt.plot(pose[0], pose[1],"bo")

    
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('Initial Positions of Agents 1 and 2')
    plt.legend()
    plt.grid(True)
    plt.show()

log_file_path = 'fast.txt'
plot_trajectory(log_file_path)