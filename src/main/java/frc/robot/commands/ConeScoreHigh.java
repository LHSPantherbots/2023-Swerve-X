// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeScoreHigh extends SequentialCommandGroup {
  /** Creates a new ConeScoreHigh. */
  public ConeScoreHigh(ElevatorSubsystem elevator, CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(elevator::resetController, elevator),
      new InstantCommand(crossSlide::resetController, crossSlide),
      new InstantCommand(intakePivot::resetController, intakePivot),
      new RunCommand(elevator::setHeightMid, elevator)
          .alongWith(new RunCommand(crossSlide::setPositionOut,crossSlide)),
      new RunCommand(intakePivot::setPositionintakeCone, intakePivot)
        
        


    );
  }
}
