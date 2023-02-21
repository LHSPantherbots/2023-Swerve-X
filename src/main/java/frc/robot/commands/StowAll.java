// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowAll extends SequentialCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public StowAll(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new InstantCommand(intakePivot::resetController, intakePivot),
      
      //Using new functional command (try this)
      new FunctionalCommand(
        // Reset encoders on command start
        intakePivot::resetController,
        // Start driving forward at the start of the command
        () -> intakePivot.setPositionStow(),
        // Stop driving at the end of the command
        interrupted -> intakePivot.closedLoopIntakePivot(),
        // End the command when the robot's driven distance exceeds the desired value
        () -> intakePivot.isAtPosition(),
        // Require the drive subsystem
        intakePivot
      ),
      
      new FunctionalCommand(
        // Reset encoders on command start
        crossSlide::resetController,
        // Start driving forward at the start of the command
        () -> crossSlide.setPositionStow(),
        // Stop driving at the end of the command
        interrupted -> crossSlide.closedLoopCrossSlide(),
        // End the command when the robot's driven distance exceeds the desired value
        () -> crossSlide.isAtPosition(),
        // Require the drive subsystem
        crossSlide
      )








      //new RunCommand(intakePivot::setPositionStow, intakePivot).withTimeout(1.0),
      //new InstantCommand(crossSlide::resetController, crossSlide),
      //.alongWith(new InstantCommand(intakePivot::resetController, intakePivot)),
      //new RunCommand(crossSlide::setPositionStow, crossSlide)
      //.alongWith(new RunCommand(intakePivot::setPositionStow, intakePivot))
      // new RunCommand(crossSlide::setPositionIntake, crossSlide).until(() -> crossSlide.isAtPosition()),

      
    );
  }
}
