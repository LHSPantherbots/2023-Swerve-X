// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeScoreHigh extends SequentialCommandGroup {
  /** Creates a new IntakeCubeCommand. */
  public CubeScoreHigh(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new ElevatorCmd(Position.CUBE_SCORE_HIGH, elevatorSubsystem),
            new SequentialCommandGroup(
                new CrossSlideCmd(Position.STOW, crossSlide, false).withTimeout(0.25),
                new CrossSlideCmd(Position.CUBE_SCORE_HIGH, crossSlide, false)),
            new IntakePivotCmd(Position.STOW, intakePivot, false)),
        new ParallelRaceGroup(
            new IntakePivotCmd(Position.CUBE_SCORE_HIGH, intakePivot),
            new ElevatorCmd(Position.CUBE_SCORE_HIGH, elevatorSubsystem, false),
            new CrossSlideCmd(Position.CUBE_SCORE_HIGH, crossSlide, false)));
  }
}
