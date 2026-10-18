import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Show Gazebo GUI'
    )

    # Recuperiamo il percorso assoluto della cartella install in modo diretto
    pkg_share_dir = get_package_share_directory('iiwa_description')
    world_path = os.path.join(pkg_share_dir, 'gazebo', 'worlds', 'empty.world')

    # Prepariamo le variabili d'ambiente prendendole direttamente dal tuo terminale corrente
    env_context = os.environ.copy()
    
    # Aggiungiamo i percorsi dei modelli per sicurezza anche qui
    my_models_path = os.path.join(pkg_share_dir, 'gazebo', 'models')
    gazebo_path = os.path.join(pkg_share_dir, 'gazebo')
    
    if 'IGN_GAZEBO_RESOURCE_PATH' in env_context:
        env_context['IGN_GAZEBO_RESOURCE_PATH'] = f"{my_models_path}:{gazebo_path}:{env_context['IGN_GAZEBO_RESOURCE_PATH']}"
    else:
        env_context['IGN_GAZEBO_RESOURCE_PATH'] = f"{my_models_path}:{gazebo_path}"

    # Eseguiamo il processo iniettando a forza l'ambiente del terminale funzionante
    gz_sim_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        env=env_context  # <--- Questo costringe ROS 2 a usare il tuo ambiente!
    )

    return LaunchDescription([
        gui_arg,
        gz_sim_launch
    ])
