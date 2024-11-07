from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    urdf = '/home/aryavarta/ros_final/src/lab4/urdf/three_wheeled_robot.urdf'
    # rviz_config_file=os.path.join(package_dir,'config.rviz')

    joint_state_publisher_node = Node( 
    package="joint_state_publisher_gui", 
    executable="joint_state_publisher_gui", 
    name="joint_state_publisher_gui", 
    output="screen", 
    arguments=[urdf] 
    ) 
    robot_state_publisher_node = Node( 
    package="robot_state_publisher", 
    executable="robot_state_publisher", 
    name="robot_state_publisher", 
    output="screen", 
    arguments=[urdf] 
    ) 
    rviz_node = Node( 
    package="rviz2", 
    executable="rviz2", 
    name="rviz2", 
    output="screen" 
    ) 
    nodes_to_run = [ 
    joint_state_publisher_node, 
    robot_state_publisher_node, 
    rviz_node 
] 
    return LaunchDescription(nodes_to_run)