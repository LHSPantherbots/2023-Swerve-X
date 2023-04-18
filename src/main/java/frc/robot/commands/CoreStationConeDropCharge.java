package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Leds;
import frc.robot.util.Position;

public class CoreStationConeDropCharge extends SequentialCommandGroup {

  public CoreStationConeDropCharge(
      ElevatorSubsystem elevator,
      CrossSlideSubsystem crossslide,
      IntakePivotSubsystem intakepivot,
      IntakeSubsystem intake,
      DriveSubsystem drivesubsystem,
      Leds led) {
    PathPlannerTrajectory path =
        PathPlanner.loadPath("CoreStationDriveOnlyCharge", new PathConstraints(1, 1), false);
    HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put(
       // "event1",
        // new ConeIntakeGround(crossslide, intakepivot, elevator)
           //  .alongWith(new RunCommand(intake::intakeCone, intake).withTimeout(1.5))
           //  .andThen(
              //  new StowAll(crossslide, intakepivot, elevator).alongWith(new IntakeHold(intake))));

    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            drivesubsystem::getPose,
            drivesubsystem::resetOdometry,
            Constants.DriveConstants.kDriveKinematics,
            new PIDConstants(
                5.0, 0.0,
                0.0), // PID constants to correct for translation error (used to create the X and Y
            // PID controllers)
            new PIDConstants(
                3.0, 0.0,
                0.0), // PID constants to correct for rotation error (used to create the rotation
            // controller),
            drivesubsystem::setModuleStates,
            eventMap,
            true,
            drivesubsystem);
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> drivesubsystem.resetOdometry(path.getInitialPose())),
            new AutoConeHigh(elevator, crossslide, intakepivot, intake),
            autoBuilder.fullAuto(path),
            new AutoBalance2(drivesubsystem),
            new InstantCommand(drivesubsystem::restAll180, drivesubsystem),
            new ParallelCommandGroup(
                new AutoBalanceTwoShot(drivesubsystem),
                new IntakePivotCmd(Position.STOW, intakepivot, false),
                new CrossSlideCmd(Position.STOW, crossslide, false))));
  }
}
