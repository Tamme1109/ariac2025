
Developing a ROS2 Infrastructure and Control System for the ARIAC 2025 Competition
=======
ros2 launch ariac_gazebo ariac.launch.py trial_name:=kitting competitor_pkg:=ariac_kandidat dev_mode:=True

ros2 launch ariac_kandidat launch.py

to spawn random parts: add the file my_bull_randomizePartsForDataCollection.py in the package ariac_gazebo/config/trials
yo can then run the python file everytime you want to update the parts. Remember to build the ariac_gazebo package every time you've updated!


TO RUNN THE OPERATION RUNNER:
ros2 run operation_runner ops_run

kör tillsammans med agv_test.py i ariac_kandidat
