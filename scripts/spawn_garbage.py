#!/usr/bin/env python3
import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

def spawn_cubes(num_cubes):
    # Initialize the ROS node
    rospy.init_node('spawn_garbage_cubes')
    
    # Wait for the Gazebo spawn service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    # Initialize a publisher for garbage positions
    garbage_pub = rospy.Publisher('/garbage_positions', PoseStamped, queue_size=5)
    
    # Define cube and runway parameters
    cube_size = 0.3  # 30cm cube
    runway_length = 100.0
    runway_width = 10.0
    
    # SDF template for a cube (with green color)
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
    
    rate = rospy.Rate(1)  # 1 Hz loop rate to give time for publishing
    
    for i in range(min(num_cubes, 5)):  # Maximum 5 cubes allowed
        # Randomly generate cube positions within runway bounds
        x_position = random.uniform(-runway_length / 2, runway_length / 2)
        y_position = random.uniform(-runway_width / 2, runway_width / 2)
        z_position = 0.05  # Small offset so cube sits above the ground
        
        # Define the initial pose for the cube
        pose = Pose()
        pose.position = Point(x_position, y_position, z_position)
        pose.orientation = Quaternion(0, 0, 0, 1)
        
        # Fill in the SDF template with cube-specific values
        sdf = sdf_template.format(name=f"cube_{i+1}", size=cube_size)
        
        try:
            # Spawn the cube in Gazebo
            spawn_model(model_name=f"cube_{i+1}",
                        model_xml=sdf,
                        robot_namespace='/',
                        initial_pose=pose,
                        reference_frame="world")
            
            rospy.loginfo("Spawned Cube {}: X={:.2f}, Y={:.2f}, Z={:.2f}".format(i+1, x_position, y_position, z_position))
            
            # Publish the cube's position so that other nodes can use it
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose = pose
            garbage_pub.publish(pose_stamped)
            
        except Exception as e:
            rospy.logerr("Failed to spawn cube {}: {}".format(i+1, e))
        
        rate.sleep()
    
    rospy.loginfo("Successfully spawned {} garbage cubes.".format(min(num_cubes, 5)))

if __name__ == "__main__":
    try:
        num_cubes = int(input("Enter the number of cubes to spawn (1-5): "))
        if 1 <= num_cubes <= 5:
            spawn_cubes(num_cubes)
        else:
            print("Please enter a number between 1 and 5.")
    except rospy.ROSInterruptException:
        pass
