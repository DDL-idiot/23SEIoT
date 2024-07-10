import pybullet as p
import pybullet_data
import math
import matplotlib.pyplot as plt

def check_pos(Pos, goal, bias):
    """
    Check if pos is at goal with bias
    Args:
    Pos: Position to be checked, [x, y]
    goal: goal position, [x, y]
    bias: bias allowed
    Returns:
    True if pos is at goal, False otherwise
    """
    return goal[0] + bias > Pos[0] > goal[0] - bias and goal[1] + bias > Pos[1] > goal[1] - bias

def goto(agent, goal_x, goal_y, trajectory):
    dis_th = 0.01
    while True:
        basePos = p.getBasePositionAndOrientation(agent)
        current_x, current_y = basePos[0][0], basePos[0][1]
        pos = [current_x, current_y]
        trajectory.append(pos)  # Store the position for plotting the trajectory

        if check_pos(pos, [goal_x, goal_y], dis_th):
            break

        current_orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((goal_y - current_y), (goal_x - current_x))

        # Normalize angles
        current_orientation = (current_orientation + 2 * math.pi) % (2 * math.pi)
        goal_direction = (goal_direction + 2 * math.pi) % (2 * math.pi)

        # Calculate angular difference
        theta = (goal_direction - current_orientation + math.pi) % (2 * math.pi) - math.pi

        # Proportional control for linear and angular velocities
        k_linear = 1
        k_angular = 0.5
        linear = k_linear * math.cos(theta)
        angular = k_angular * theta

        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=10)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=10)

        p.stepSimulation()

def plot_trajectory(trajectory):
    x, y = zip(*trajectory)
    plt.plot(x, y, marker='o')
    plt.title("Robot Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid(True)
    plt.show()

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)

startPosition = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-89.9, cameraTargetPosition=[0, 0, 0])

trajectory = []  # List to store trajectory points
goto(boxId, 0, 2, trajectory)
goto(boxId, 2, 2, trajectory)

plot_trajectory(trajectory)  # Plot the trajectory after simulation
