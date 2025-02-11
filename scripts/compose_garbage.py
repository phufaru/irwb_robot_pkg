#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties

def remove_garbage():
    rospy.init_node('compose_garbage')

    # Wait for services to be available
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/gazebo/get_world_properties')

    # Initialize the service proxies
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    try:
        # Get the list of models in the world
        world_properties = get_world_properties()
        models_in_world = world_properties.model_names

        # Filter for garbage cubes (we assume they are named cube_1, cube_2, etc.)
        cubes_to_remove = [model for model in models_in_world if model.startswith("cube_")]

        if not cubes_to_remove:
            rospy.loginfo("No garbage cubes found to remove.")
            return

        rospy.loginfo(f"Found {len(cubes_to_remove)} cube(s) to remove: {', '.join(cubes_to_remove)}")

        # Delete each cube
        for cube_name in cubes_to_remove:
            try:
                delete_model(cube_name)
                rospy.loginfo(f"Successfully removed {cube_name}")
            except Exception as e:
                rospy.logerr(f"Failed to remove {cube_name}: {e}")

    except Exception as e:
        rospy.logerr(f"Error in retrieving or deleting models: {e}")

if __name__ == "__main__":
    try:
        remove_garbage()
    except rospy.ROSInterruptException:
        pass
