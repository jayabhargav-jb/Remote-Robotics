# Setup
- Build the docker container and run, you might have to change the compose.yaml file to change the volume directory.
- Inside the container, source the ros installation first
``` source /opt/ros/humble/setup.bash ```
- Then use ``` colcon build ``` in the "ros_server" directory to build the workspace
- Source your workspace after building using ``` source ./install/setup.bash ```
- Check if the ``` /scan ``` topic is being published, then run ``` ros2 run laser_to_image laser_to_image_node ```