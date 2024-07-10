import yaml
import matplotlib.pyplot as plt

# Load the YAML file
file_path = 'cbs_output.yaml'  # Replace with the correct path to your YAML file
with open(file_path, 'r') as file:
    cbs_output = yaml.safe_load(file)

# Extracting the paths of the two robots
robot_paths = cbs_output["schedule"]

# Plotting the paths of each robot
plt.figure(figsize=(10, 10))

for robot_id, path in robot_paths.items():
    x_coords = [step["x"] for step in path]
    y_coords = [step["y"] for step in path]

    plt.plot(x_coords, y_coords, marker='o', linestyle='-', label=f'Robot {robot_id}')

plt.title("Paths of Two Simulated Robots")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.legend()
plt.grid(True)
plt.show()
