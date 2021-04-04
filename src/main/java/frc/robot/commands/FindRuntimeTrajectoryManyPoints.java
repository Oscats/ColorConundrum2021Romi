// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Waypoints;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more

// information, see: https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FindRuntimeTrajectoryManyPoints extends CommandBase {
  /** Creates a new PlotTrajectory. */
  Drivetrain m_drivetrain;
  Trajectory m_Trajectory;
  Trajectory runTimeTrajectory;
  List<Translation2d> m_list;
  List<Pose2d> m_pose;
  //List <Translation2d> waypoints;
  Waypoints m_waypoints;
  TrajectoryConfig config;

  public FindRuntimeTrajectoryManyPoints(Trajectory trajectory, Drivetrain drivetrain, Waypoints waypoints) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_Trajectory = trajectory;
    m_waypoints = waypoints;
    m_pose = m_waypoints.getPathAPoses();
    m_list = m_waypoints.getPathAWaypoints();




    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                       DriveConstants.kvVoltSecondsPerMeter, 
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    
    config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    //We are using the lists in the Waypoints file to set this up.
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //setWayPoints();

	runTimeTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(Units.inchesToMeters(SmartDashboard.getNumber("Start PoseX", 0)), 
    Units.inchesToMeters(SmartDashboard.getNumber("Start PoseY", 0)), 
    new Rotation2d(SmartDashboard.getNumber("Start PoseAngle(in Degrees)", 0))),
    m_waypoints.getWaypoints()
    ,
        new Pose2d(Units.inchesToMeters(SmartDashboard.getNumber("End PoseX", 0)), 
          Units.inchesToMeters(SmartDashboard.getNumber("End PoseY", 0)), 
          new Rotation2d(SmartDashboard.getNumber("End PoseAngle(in Degrees)", 0))),
        config);
    m_drivetrain.getField().getObject("trajectory").setPoses(
      runTimeTrajectory.getStates().stream()
              .map(state -> state.poseMeters)
              .collect(Collectors.toList()));
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
