import launch
import launch_ros.actions
import os

def generate_launch_description():
    return launch.LaunchDescription([

        launch.actions.SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{name}]: {message}'
        ),
        # ✅ controller 실행 (새 터미널)
        launch.actions.ExecuteProcess(
            cmd=['gnome-terminal', '--title=Controller Terminal', '--', 'bash', '-c', 
                 'source ' + os.environ['HOME'] + '/ros2_ws/install/setup.bash && ros2 run jet_walking controller; exit'],
            output='screen',
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]  # 🔹 controller 종료 시 전체 종료
        ),
        # ✅ sensor 실행 (새 터미널)
        launch.actions.ExecuteProcess(
            cmd=['gnome-terminal', '--title=Sensor Terminal', '--', 'bash', '-c', 
                 'source ' + os.environ['HOME'] + '/ros2_ws/install/setup.bash && ros2 run jet_walking sensor; exit'],
            output='screen'
        ),
        # ✅ microstrain 드라이버 실행 (환경변수 강제 설정)
        launch.actions.ExecuteProcess(
            cmd=['ros2 launch microstrain_inertial_driver microstrain_launch.py'],
            shell=True,
            output='screen'
        )
    ])


