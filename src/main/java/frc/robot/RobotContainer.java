// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FindNewTrajectoryRuntime;
import frc.robot.commands.PlotTrajectory;
import frc.robot.commands.FindRuntimeTrajectoryManyPoints;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final Drivetrain m_drivetrain = new Drivetrain();
   private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
 
   // Assumes a gamepad plugged into channnel 0
   private final Joystick m_controller = new Joystick(0);
 
   // Create SmartDashboard chooser for autonomous routines
   private final SendableChooser<Command> m_chooser = new SendableChooser<>();
 
   //Create an instance of our waypoints
   private Waypoints m_waypoints = new Waypoints();
 
   //Make our Trajectory local so we can pass it to the plotter.
   Trajectory exampleTrajectory;

   private double[] defaultCoordinates = {0.0, 5.0};
   private double[] defaultPoseCoordinates = {0,0,180};
   
 
   // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
   // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
   // By default, the following are available (listed in order from inside of the board to outside):
   // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
   // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
   // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
   // - PWM 2 (mapped to Arduino Pin 21)
   // - PWM 3 (mapped to Arduino Pin 22)
   //
   // Your subsystem configuration should take the overlays into account
 
   /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand(Waypoints m_waypoints) {
    List<Pose2d> m_pose = m_waypoints.getPathAPoses();
    List<Translation2d>m_list = m_waypoints.getPathAWaypoints();
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                       DriveConstants.kvVoltSecondsPerMeter, 
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        m_pose.get(0),
        m_list,
        m_pose.get(1),
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

    //m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());


    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * You can also configure smart dashboard buttons here.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand(m_waypoints));
    m_chooser.addOption("Plot Points",new PlotTrajectory(exampleTrajectory, m_drivetrain,m_waypoints));
    m_chooser.addOption("Plot Points at Runtime",new FindNewTrajectoryRuntime(exampleTrajectory, m_drivetrain, m_waypoints));
    m_chooser.addOption("Plot Many Points at Runtime", new FindRuntimeTrajectoryManyPoints(exampleTrajectory, m_drivetrain, m_waypoints));
    //Setup starting pose entry for runtime trajectory...
    SmartDashboard.putNumber("Start PoseX", 0);
    SmartDashboard.putNumber("Start PoseY", 0);
    SmartDashboard.putNumber("Start PoseAngle(in Degrees)", 0);
    SmartDashboard.putNumberArray("starting pose", defaultPoseCoordinates);
    //Setup waypoint entry for runtime trajectory...
    SmartDashboard.putNumber("PointX", 5);
    SmartDashboard.putNumber("PointY", 5);
    
    //Setup end pose entry for runtime trajectory...
    SmartDashboard.putNumber("End PoseX", 10);
    SmartDashboard.putNumber("End PoseY", 10);
    SmartDashboard.putNumber("End PoseAngle(in Degrees)", 180);

    //Setup waypoint entry for runtime trajectory...
    SmartDashboard.putNumber("PointX", 5);
    SmartDashboard.putNumber("PointY", 5);

    //Maybe we can use arrays to streamline the waypoint creation and make many waypoints...
    SmartDashboard.putNumberArray("Waypoint 1", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 2", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 3", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 4", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 5", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 6", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 7", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 8", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 9", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 910", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 911", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 912", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 913", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 914", defaultCoordinates);
    SmartDashboard.putNumberArray("Waypoint 915", defaultCoordinates);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -(m_controller.getRawAxis(1))/1.5, () -> (m_controller.getRawAxis(0))/1.5);
  }
}

