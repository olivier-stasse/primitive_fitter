from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    # Build parameters
    package_name ='nimbro_primitive_fitter'
    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory(package_name)
    print('package_share_directory: ' + package_share_directory)
    urdf_filename = package_share_directory +  \
      '/models/ur10_stl.urdf'
    print('urdf_filename : ' + urdf_filename)
    output_filename = package_share_directory +  \
      '/models/ur10_stl_fitcapsule.urdf.xacro'
    print('output_filename : ' + output_filename) 

    urdf_filename_launch_arg = DeclareLaunchArgument(
        'urdf_filename', default_value=TextSubstitution(text=urdf_filename)
    )
    output_filename_launch_arg = DeclareLaunchArgument(
        'output_filename', default_value=TextSubstitution(text=output_filename)
    )
    fit_shape_launch_arg = DeclareLaunchArgument(
        'fit_shape', default_value=TextSubstitution(text='capsule')
    )
    use_fitter_launch_arg = DeclareLaunchArgument(
        'use_fitter', default_value=TextSubstitution(text='true')
    )
    use_inertia_launch_arg = DeclareLaunchArgument(
        'use_inertia', default_value=TextSubstitution(text='true')
    )
    mesh_type_launch_arg = DeclareLaunchArgument(
        'mesh_type', default_value=TextSubstitution(text='collision')
    )
   
    return LaunchDescription([
        urdf_filename_launch_arg,
        output_filename_launch_arg,
        fit_shape_launch_arg,
        use_fitter_launch_arg,
        use_inertia_launch_arg,
        mesh_type_launch_arg,
        Node(
            package='nimbro_primitive_fitter',
            executable='nimbro_primitive_fitter',
            name='nimbro_primitive_fitter',
            parameters=[{
                'urdf_filename': LaunchConfiguration('urdf_filename'),
                'output_filename': LaunchConfiguration('output_filename'),
                'fit_shape': LaunchConfiguration('fit_shape'),
                'use_fitter': LaunchConfiguration('use_fitter'),
                'use_inertia': LaunchConfiguration('use_inertia'),
                'mesh_type': LaunchConfiguration('mesh_type'),
            }]
        ),
    ])
