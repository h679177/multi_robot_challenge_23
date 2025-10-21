import launch
import launch_ros

def generate_launch_description():
    namespace_node1 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='tb3_0_robotHandler'
    )
    
    namespace_node2 = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='tb3_1_robotHandler'
    )

    leader = launch_ros.actions.Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='tb3_leader'

    return launch.LaunchDescription([
        namespace_node1,
        namespace_node2,
        leader
    ])