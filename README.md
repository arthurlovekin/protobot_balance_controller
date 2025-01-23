This is a custom PID controller to balance the protobot.
The protobot has two modes of operation:
1. Diff-drive mode. The protobot_balance_controller is off and the 
    diff_drive_controller receives velocity commands directly from the /cmd_vel topic
2. Balance-mode. The protobot_balance_controller is on, and recieves a reference pitch from the state estimation node, and velocity commands from the /cmd_vel topic. It outputs a linear and angular velocity to the diff_drive_controller, which is set to chained mode. 

Inside the protobot_balance_controller, there is PID loop that tries to reduce the error between the pitch of the robot and the setpoint pitch. For stationary balancing, we set pitch_setpoint = pitch_0, the pitch of the robot when it is at its (unstable) balancing point.To move while balancing, we recognize that leaning forward will cause the robot to move forward. Thereforw, we mangle units and make the pitch_setpoint proportional to v_x, the linear term of /cmd_vel: 

pitch_setpoint = pitch_0 + setpoint_constant*v_x

There is also a node that will monitor the pitch of the robot as it drives, and if the pitch exceeds a certain pitch_threshold, then it will turn on the protobot_balance_controller. This way, the user can drive in diff-drive mode and trigger a balance by manually accelerating quickly and making the robot do a wheelie. Balance mode reverts to diff-drive mode if the robot falls back down or if a kill button is pressed on the joystick.

The angular velocity gets linearly scaled down but otherwise passes through unchanged to the diff_drive_controller.

See the test/config folder for examples of these parameters with their description.

Tutorials:
[ros2_control Writing a new controller](https://control.ros.org/rolling/doc/ros2_controllers/doc/writing_new_controller.html)
[ros2_control_demos Example 7: Full Tutorial with 6DOF robot](https://control.ros.org/rolling/doc/ros2_control_demos/example_7/doc/userdoc.html)
[ROS2 Plguinlib tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
[ROS2 generate_parameters_library](https://github.com/PickNikRobotics/generate_parameter_library)

