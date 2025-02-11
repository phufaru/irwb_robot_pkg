#!/usr/bin/env python3

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_cubes(num_cubes):
    rospy.init_node('spawn_garbage_cubes')

    # Wait for gazebo_ros service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Cube properties
    cube_size = 0.3  # 30cm
    runway_length = 100.0
    runway_width = 10.0

    # SDF template for the cube (green material)
    sdf_template = """
    <sdf version="1.6">
      <model name="{name}">
        <static>false</static>
        <link name="cube_link">
          <collision name="collision">
            <geometry>
              <box>
                <size>{size} {size} {size}</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>{size} {size} {size}</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Green</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

    # Keep track of cube positions
    cube_positions = []

    for i in range(min(num_cubes, 5)):  # Maximum 5 cubes
        # Randomly generate position within runway bounds
        x_position = random.uniform(-runway_length / 2, runway_length / 2)
        y_position = random.uniform(-runway_width / 2, runway_width / 2)
        z_position = 0.05  # Small offset so the cube is above the runway

        # Define the pose
        pose = Pose()
        pose.position = Point(x_position, y_position, z_position)
        pose.orientation = Quaternion(0, 0, 0, 1)

        # Fill in the SDF with cube-specific data
        sdf = sdf_template.format(name=f"cube_{i+1}", size=cube_size)

        # Spawn the cube
        try:
            spawn_model(model_name=f"cube_{i+1}",
                        model_xml=sdf,
                        robot_namespace='/',
                        initial_pose=pose,
                        reference_frame="world")

            # Store and display the cube's position
            cube_positions.append((x_position, y_position, z_position))
            rospy.loginfo(f"Cube {i+1}: X={x_position:.2f}, Y={y_position:.2f}, Z={z_position:.2f}")

        except Exception as e:
            rospy.logerr(f"Failed to spawn cube {i+1}: {e}")

    # Print positions of all cubes
    print("\nSpawned Cube Positions:")
    for i, pos in enumerate(cube_positions):
        print(f"Cube {i+1}: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}")

if __name__ == "__main__":
    try:
        # Ask the user how many cubes they want to spawn
        num_cubes = int(input("Enter the number of cubes to spawn (1-5): "))
        if 1 <= num_cubes <= 5:
            spawn_cubes(num_cubes)
        else:
            print("Please enter a number between 1 and 5.")
    except rospy.ROSInterruptException:
        pass