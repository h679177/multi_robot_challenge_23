import launch
import launch_ros

def generate_launch_description():
    namespace_node_1 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='tb3_1'
    )
    
    namespace_node_2 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='tb3_2'
    )

    leader = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='tb3_leader'
    )
    
    return launch.LaunchDescription([
        namespace_node_1,
        namespace_node_2,
        leader
    ])
