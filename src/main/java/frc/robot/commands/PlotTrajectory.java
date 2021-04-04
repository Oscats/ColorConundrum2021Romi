// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Waypoints;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class PlotTrajectory extends CommandBase {
  /** Creates a new PlotTrajectory. */
  Drivetrain m_drivetrain;
  Trajectory m_Trajectory;
  Trajectory exampleTrajectory;
  List<Translation2d> m_list;
  List<Pose2d> m_pose;
  Waypoints m_waypoints;

  public PlotTrajectory(Trajectory trajectory, Drivetrain drivetrain, Waypoints waypoints) {
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

    
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    //We are using the lists in the Waypoints file to set this up.
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        m_pose.get(0),
        m_list,
        m_pose.get(1),
        config);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.getField().getObject("trajectory").setPoses(
      exampleTrajectory.getStates().stream()
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
