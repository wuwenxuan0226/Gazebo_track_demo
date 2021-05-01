## How to use Gazebo branch
### Creat a map:
#### roslaunch jetracer_description gazebo_map.launch 
### Spawn a jetracer model:
#### roslaunch jetracer_description spawn.launch
### Launch the Gazebo line following contrl loop:
#### roslaunch jetracer_line_detect gazebo_line_following.launch
### Run the constant steering angle open loop test:
#### roslaunch jetracer_logger open_loop_drive.launch
### Run the stright open loop test:
#### roslaunch jetracer_logger open_loop_drive_stright.launch
