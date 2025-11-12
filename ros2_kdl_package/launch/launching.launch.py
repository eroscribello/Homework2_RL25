from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface', 
        default_value='velocity',
        description='Select controller: position, velocity or effort'
    )
    
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', 
        default_value='velocity_ctrl',
        description='Select velocity controller: velocity_ctrl or velocity_ctrl_null'
    )

    cmd_interface_val = LaunchConfiguration('cmd_interface')
    ctrl_val = LaunchConfiguration('ctrl')

    # 3. Definizione del Nodo
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        parameters=[
            # Carica il file YAML originale
            PathJoinSubstitution([
                FindPackageShare('ros2_kdl_package'),   
                'config',
                'kdl_params.yaml'
            ]),
            {'cmd_interface': cmd_interface_val},
            {'ctrl': ctrl_val}
        ]
    )
    nodes_to_start = [
        cmd_interface_arg,  
        ctrl_arg,           
        ros2_kdl_node,      
    ]
        
    return LaunchDescription(nodes_to_start)