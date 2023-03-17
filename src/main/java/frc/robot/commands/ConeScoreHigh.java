// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeScoreHigh extends SequentialCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public ConeScoreHigh(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new ElevatorCmd(Position.CONE_SCORE_HIGH, elevatorSubsystem),
            new CrossSlideCmd(Position.CONE_SCORE_HIGH, crossSlide, false),
            new IntakePivotCmd(Position.STOW, intakePivot, false)),
        new ParallelRaceGroup(
            new IntakePivotCmd(Position.CONE_SCORE_HIGH, intakePivot),
            new ElevatorCmd(Position.CONE_SCORE_HIGH, elevatorSubsystem, false),
            new CrossSlideCmd(Position.CONE_SCORE_HIGH, crossSlide, false)),
            new InstantCommand(() -> RobotContainer.robotState.setPosition(Position.CONE_SCORE_HIGH))

            );

  }
}
