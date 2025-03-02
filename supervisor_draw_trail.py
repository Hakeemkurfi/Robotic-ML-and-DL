from controller import Supervisor

# Initialize the Supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

# Get the robot node by its DEF name
robot_node = supervisor.getFromDef("PIONEER_3DX")
if robot_node is None:
    print("Error: No DEF name 'PIONEER_3DX' found in the world file.")
    exit(1)

# Create a Trail node to visualize the path
trail_node = supervisor.getRoot().getField("children").importMFNodeFromString(-1, 'Trail { color 1 0 0 width 0.05 }')

# Main loop
while supervisor.step(time_step) != -1:
    # Get the current position of the robot
    position = robot_node.getPosition()

    # Append the current position to the trail
    trail_node.getField("trail").addSFVec3f(position)
