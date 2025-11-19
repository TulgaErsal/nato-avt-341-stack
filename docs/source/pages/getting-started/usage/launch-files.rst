Launch File Setup
==================

Base Launch Files
---------------------

Within the stack, base launch files exist which define the needed stack nodes and configuration. To interface with the autonomy stack, reference these base files.

=================   =================
ROS Version         Launch File
=================   =================
ROS 1                ``avt_341/launch/base.launch``
ROS 2                | ``avt_341/launch/base.launch.py``
                     | or ``avt_341/launch/krc_base.launch.py``
=================   =================

.. todo::

    The ROS2 ``avt_341/launch/base.launch.py`` and ``avt_341/launch/krc_base.launch.py`` launch files need to be merged.

Sample Child Launch File
--------------------------

Below is a sample minimal ROS2 launch file that extends the base launch file. The few customized settings shown are the most important, though refer to the `full parameter list <parameters.html>`_ for the complete roslaunch parameter list.


.. code-block:: shell

    def generate_launch_description():

        avt_341_dir = get_package_share_directory('avt_341')
        avt_341_launch = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'base.launch.py')),
            launch_arguments={
                # Occupancy grid configuration
                'grid_llx': '-420.0',
                'grid_lly': '-780.0',
                'grid_width': '800.0',
                'grid_height': '1600.0',
                'grid_res': '1.0',
                'grid_dilate': 'True',

                # Target vehicle speed
                'vehicle_speed': '8.0',

                # Vehicle selection
                'num_vehicles': '4',
                'vehicle_namespaces': "['agv1', 'agv2', 'cgv1', 'cgv2']",

                # Speed controller
                'throttle_kp': '0.1',
                'throttle_ki': '0.05',

                # Local planner
                'local_planner_method': 'dwa',
              }.items(),
        )

        launch_actions = [
            avt_341_launch,
            # Other launch files specific to your own setup
        ]

        return LaunchDescription(launch_actions)


