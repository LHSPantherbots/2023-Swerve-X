// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighScoreCubeBalance extends SequentialCommandGroup {

  Trajectory backupTrajectory = new Trajectory();
  SwerveControllerCommand backupSwerveCommand;

  /** Creates a new HighScoreConeBalanceAuto. */
  public AutoHighScoreCubeBalance(
      DriveSubsystem driveTrain,
      ElevatorSubsystem elevator,
      CrossSlideSubsystem crossslide,
      IntakePivotSubsystem intakepivot,
      IntakeSubsystem intake) {

    TrajectoryConfig rev_config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setReversed(true)
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.5, 0)),
            // List.of(new Translation2d(.5, .5), new Translation2d(1.0, -.5)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.5, 0.0, new Rotation2d(Math.PI)),
            // config);
            rev_config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            driveTrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    // driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    driveTrain.resetOdometryReverse(exampleTrajectory.getInitialPose());

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveTrain.resetOdometryReverse(driveTrain.getPose()), driveTrain),
        new AutoCubeHigh(elevator, crossslide, intakepivot, intake),
        swerveControllerCommand
            .raceWith(new RunCommand(elevator::closedLoopElevator, elevator))
            .raceWith(new RunCommand(crossslide::closedLoopCrossSlide, crossslide))
            .raceWith(new RunCommand(intakepivot::closedLoopIntakePivot, intakepivot)),
        new InstantCommand(driveTrain::restAll180, driveTrain),
        new AutoBalance(driveTrain));
  }
}
