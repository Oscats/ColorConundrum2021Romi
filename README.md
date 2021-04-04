# Romi Trajectory

## Description
This Repository is a modified version of the incredible work that @zhiquanyeo created to help us all learn how to use trajectories. Without him, we would be at a bit of a loss. You can access his repository [directly](https://github.com/bb-frc-workshops/romi-examples).

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in inches, and output to meters using the `Units.inchesToMeters()` method.

## Additional Hardware Required
None

## Additional Hardware Optional
Perhaps the next step to extend this further would be to add a sensor or vision and get a trajectory to a desired point on the field.

## Additional Configuration Required
- Ensure that the gyro has been [calibrated using the web UI](https://docs.wpilib.org/en/stable/docs/romi-robot/web-ui.html#imu-calibration)
- For best results, you should run a characterization on your Romi using the tool in [Zhiquanyeo's Git.](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) since there might be slight variations between Romis (due to assembly, mechanical difference, etc)
    

## Additional Code Setup
We have created a few handy convenience features that allow you to easily imput and test waypoints.
- [AddWaypoints()](src/main/java/frc/robot/Waypoints.java) Create a new list(or modify the existing one) of waypoints and starting and ending poses.
- [PlotTrajectory() Command](src/main/java/frc/robot/commands/PlotTrajectory.java) While not needing much editing, this command allows you to visualize the trajectory using the Field2d() class. You can use it to fine-tune your wyapoints and starting and ending poses. It is set as an option for the Autonomous selcetor in [RobotContainer()](src/main/java/frc/robot/RobotContainer.java). You can access it by using shuffleboard, or the Sendable Chooser in Glass or the Simulator Window.
- [FindNewTrajectoryRuntime() Command](src/main/java/frc/robot/commands/FindNewTrajectoryRuntime.java) Use the following smartdashboard entries to create a trajectory that you can move around the screen.
    - Start PoseX, Start PoseY, Start PoseAngle(in degrees) -- Makes up the start pose.
    - WaypointX, WaypointY -- Makes up the waypoint (you may modify the code to add more waypoints if you would like.)
    - End PoseX, End PoseY, End Pose Angle (in degrees) -- Makes up the ending pose.
- [FindRuntimeTrajectoryManyPoints() Command](src/main/java/frc/robot/commands/FindRuntimeTrajectoryManyPoints.java) Use the following smartdashboard entries to create a trajectory that you can move around the screen.
    - Start PoseX, Start PoseY, Start PoseAngle(in degrees) -- Makes up the start pose.
    - Waypoint(*) -- Makes up the waypoint. If left as the default x, coordinate, this does nothing, but if you change it, it will add the new coordinate to the trajectory. They go in order. So, in theory, the first point will be the first, and so on.
    - End PoseX, End PoseY, End Pose Angle (in degrees) -- Makes up the ending pose.