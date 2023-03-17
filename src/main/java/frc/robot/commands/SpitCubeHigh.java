package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Position;

public class SpitCubeHigh extends SequentialCommandGroup {

  public SpitCubeHigh(
      ElevatorSubsystem elevator,
      CrossSlideSubsystem crossslide,
      IntakePivotSubsystem intakepivot,
      IntakeSubsystem intake) {
    addCommands(
        new ParallelRaceGroup(
            new ElevatorCmd(Position.CUBE_SCORE_HIGH, elevator),
            new SequentialCommandGroup(
                new CrossSlideCmd(Position.STOW, crossslide, false).withTimeout(0.25),
                new CrossSlideCmd(Position.CUBE_SCORE_HIGH, crossslide, false)),
            new IntakePivotCmd(Position.STOW, intakepivot, false)),
        new RunCommand(intake::ejectCube, intake).withTimeout(0.5),
        new InstantCommand(() -> RobotContainer.robotState.setPosition(Position.CUBE_SCORE_HIGH)));

    // RobotContainer.robotState.setPosition(Position.CUBE_SCORE_HIGH);
  }
}
