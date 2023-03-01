// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeIntakeGround extends ParallelCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public ConeIntakeGround(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElevatorCmd(Position.CONE_INTAKE, elevatorSubsystem),
        new CrossSlideCmd(Position.CONE_INTAKE, crossSlide),
        new IntakePivotCmd(Position.CONE_INTAKE, intakePivot));
  }
}
