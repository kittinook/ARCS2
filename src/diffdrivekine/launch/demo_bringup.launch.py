from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    str_namespace = 'Demo'
    motor_name_list = ['left', 'right']
    init_wheel_radius = 0.1
    init_wheel_base = 0.5  
    init_Kp = 0.1
    init_Ki = 0.01
    init_Kd = 0.01
    init_U_max = 18.0

    demo_diffdrive_fk_node = Node(
        package='diffdrivekine',
        executable='diffdrive_fk.py',
        name='diffdrive_fk_node',
        namespace=str_namespace,
        output='screen',
        parameters=[
            {'wheel_radius': init_wheel_radius},
            {'wheel_base': init_wheel_base}
        ]
    )

    demo_diffdrive_ik_node = Node(
        package='diffdrivekine',
        executable='diffdrive_ik.py',
        name='diffdrive_ik_node',
        namespace=str_namespace,
        output='screen',
        parameters=[
            {'wheel_radius': init_wheel_radius},
            {'wheel_base': init_wheel_base}
        ],
        remappings=[
            ('cmd_vel', 'input/cmd_vel')
        ]
    )

    motor_sim_node = Node(
        package='motorsim',
        executable='motorsim_node.py',
        name='motor_sim_node',
        output='screen'
    )

    turtle_sim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle_sim_node',
        output='screen'
    )

    kill_turtle_service = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/kill',
            'turtlesim/srv/Kill',
            '{name: "turtle1"}'
        ],
        output='screen'
    )

    spawn_turtle_service = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/spawn',
            'turtlesim/srv/Spawn',
            '{name: "Demo"}'
        ],
        output='screen'
    )

    actions = [
        demo_diffdrive_fk_node,
        demo_diffdrive_ik_node,
        motor_sim_node,
        turtle_sim_node,
        kill_turtle_service,
        spawn_turtle_service
    ]

    for motor_name in motor_name_list:
        spawn_motor_service = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn_motor',
                'motorsim_interfaces/srv/MotorSpawn',
                '{name: "/' + str_namespace + '/' + motor_name + '"}'
            ],
            output='screen'
        )
        actions.append(spawn_motor_service)

        controller_node = Node(
            package='motorsim',
            executable='controller_node.py',
            name= 'motor_controller',
            namespace=str_namespace + '/' + motor_name,
            output='screen',
            parameters=[
                {'Kp': init_Kp},
                {'Ki': init_Ki},
                {'Kd': init_Kd},
                {'U_max': init_U_max}
            ]
        )
        actions.append(controller_node)

    return LaunchDescription(actions)
