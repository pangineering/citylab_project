import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()
    
    # Create a Node object for the test_service_node
    test_node = Node(
        package='robot_patrol',
        executable='test_service_node',
        output='screen',
        emulate_tty=True,
    )
    
    # Add the test_node to the LaunchDescription
    ld.add_action(test_node)
    
    # Return the LaunchDescription
    return ld