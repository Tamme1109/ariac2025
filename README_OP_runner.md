steps for running the operation runner:
1. make sure you have built all the relevant packages (actions_operation_runner, ariac_kandidat, and operation_runner)
2. open one terminal and launch the simulation (I use this one):
    ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_kandidat dev_mode:=True

3. in another teminal run the agv_test.py script:
     ros2 run ariac_kandidat agv_test.py

4. run the operation runner with:
    ros2 run operation_runner ops_run

Any questions? DM me.