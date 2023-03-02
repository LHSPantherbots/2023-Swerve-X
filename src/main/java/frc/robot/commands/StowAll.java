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
public class StowAll extends SequentialCommandGroup {
  // Stows all subsytems for driving.  Lifts the intake pivot then drops the elevator and pulls in
  // the crosslide at the same time
  public StowAll(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Lifts the pivot
        new ParallelRaceGroup(
            new IntakePivotCmd(Position.STOW, intakePivot),
            new ElevatorCmd(Position.HOLD, elevator, false),
            new CrossSlideCmd(Position.STOW, crossSlide, false)),
        new ParallelRaceGroup(
            new ElevatorCmd(Position.STOW, elevator),
            new CrossSlideCmd(Position.STOW, crossSlide, false),
            new IntakePivotCmd(Position.STOW, intakePivot, false)));
  }
}
