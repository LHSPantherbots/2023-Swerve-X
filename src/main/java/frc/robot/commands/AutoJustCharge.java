// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoJustCharge extends SequentialCommandGroup {
  double xSpeed;

  public AutoJustCharge(DriveSubsystem driveSubsystem) {
    this(driveSubsystem, false);
  }
  ;
  /** Creates a new AutoBalance. */
  public AutoJustCharge(DriveSubsystem driveSubsystem, Boolean reversed) {
    if (reversed) {
      this.xSpeed = -0.6;
    } else {
      this.xSpeed = 0.6;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPlannerTrajectory path =
        PathPlanner.loadPath("AutoJustCharge", new PathConstraints(3, 2), false);
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetOdometry,
            Constants.DriveConstants.kDriveKinematics,
            new PIDConstants(
                5.0, 0.0,
                0.0), // PID constants to correct for translation error (used to create the X and Y
            // PID controllers)
            new PIDConstants(
                3.0, 0.0,
                0.0), // PID constants to correct for rotation error (used to create the rotation
            // controller),
            driveSubsystem::setModuleStates,
            eventMap,
            true,
            driveSubsystem);
    addCommands(
        new ParallelRaceGroup(
            new InstantCommand(() -> driveSubsystem.resetOdometry(path.getInitialPose())),
            autoBuilder.fullAuto(path),
            new InstantCommand(driveSubsystem::restAll180, driveSubsystem)));
  }
}
