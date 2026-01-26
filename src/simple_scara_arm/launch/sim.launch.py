from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


# This launch file launches the SCARA arm alone
def generate_launch_description():
    
    # Get the directories for the package and urdf robot description
    pkg = get_package_share_directory('simple_scara_arm')
    
    urdf_path = os.path.join(pkg, 'urdf', 'myrobot2.urdf')
    
    world_file = os.path.join(pkg,'worlds','inspection.world.sdf')

    robot_description = open(urdf_path).read()
    
    # Ensure Gazebo can find the ros2 control system plugins
    set_gz_plugin = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    # start gazebo with inspection world
    gazebo_inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )


    # start empty gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")]
    )
    
    # TBD
    gazebo_launch_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r inspection.world")]
    )
    
    # 1 Start Gazebo minimal manually wtih no launch file
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf'],
        output='screen'
    )
    

    # 2 Robot State publisher Node
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 3 Spawn the robot into Gazebo
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'simple_scara',
            '-string', robot_description,
            '-x', '0', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )
    
    # 4 Start ros2_control manager
    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        {'robot_description': robot_description}],
    output='screen'
)


    # 5 Spawn controllers (these require the controller_manager to exist)
    
    # Joint State Broadcaster Controller Node
    jsb = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # Robot Arm Position Controller Node
    arm_ctrl = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm_position_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )
    
     # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription([
        # set_gz_plugin,
        # gazebo,
        gazebo_inspection_launch,
        # gazebo_launch,
        gazebo_bridge,
        rsp,
        TimerAction(period=2.0, actions=[spawn]),
        # TimerAction(period=8.0, actions=[controller_manager]),
        TimerAction(period=4.0, actions=[jsb]),
        TimerAction(period=6.0, actions=[arm_ctrl]),
    ])
