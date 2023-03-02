package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoScoreTwoBalance extends SequentialCommandGroup {

  public AutoScoreTwoBalance(
      ElevatorSubsystem elevator,
      CrossSlideSubsystem crossslide,
      IntakePivotSubsystem intakepivot,
      IntakeSubsystem intake,
      DriveSubsystem driveTrain) {
    addCommands(
        new InstantCommand(() -> driveTrain.resetOdometryReverse(driveTrain.getPose()), driveTrain),
        new AutoConeHigh(elevator, crossslide, intakepivot, intake),
        // new RunCommand(() -> intake.ejectCone(), intake).withTimeout(.25),
        new ParallelRaceGroup(
            new StowAll(crossslide, intakepivot, elevator), new PPAuto("PickUpCubeTop")),
        new CubeIntakeGround(crossslide, intakepivot, elevator),
        new RunCommand(() -> intake.intakeCube(), intake).withTimeout(0.5),
        new InstantCommand(() -> intake.stopIntake(), intake),
        new ParallelRaceGroup(
            new StowAll(crossslide, intakepivot, elevator), new PPAuto("ScoreCubeTop")),
        new AutoCubeHigh(elevator, crossslide, intakepivot, intake),
        new ParallelRaceGroup(
            new StowAll(crossslide, intakepivot, elevator), new PPAuto("TopToChargeStation")),
        new RunCommand(() -> driveTrain.drive(0.6, 0.0, 0.0, true), driveTrain)
            .until(() -> driveTrain.isAtAutoBalanceAngle()),
        new RunCommand(driveTrain::xWheels, driveTrain));
  }
}
