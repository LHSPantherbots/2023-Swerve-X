// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OLD_Code;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowAll_OLD extends SequentialCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public StowAll_OLD(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new InstantCommand(intakePivot::resetController, intakePivot),
      
      //Using new functional command (try this)
      new FunctionalCommand(
        // Reset controller on command start
        intakePivot::resetController,
        // Start movint intake pivot to stow position
        () -> intakePivot.setPositionStow(),
        // at the end of the command call the closed loop intake to hold the setpoint position
        interrupted -> intakePivot.closedLoopIntakePivot(),
        // End the command when the intakePivot is at position
        () -> intakePivot.isAtPosition(),
        // Require the intakePivot subsystem
        intakePivot
      ).raceWith(new RunCommand(crossSlide::closedLoopCrossSlide, crossSlide))
      .raceWith(new RunCommand(elevatorSubsystem::closedLoopElevator, elevatorSubsystem)),
      
      new FunctionalCommand(
        // Reset controller on command start
        crossSlide::resetController,
        // run the crossSlide to the stow position
        () -> crossSlide.setPositionStow(),
        // at the end of the command call the closed loop cross slide to hold the setpoint
        interrupted -> crossSlide.closedLoopCrossSlide(),
        // End the command when intakePivot is at Position
        () -> crossSlide.isAtPosition(),
        // Require the crossSlide subsystem
        crossSlide
      ).raceWith(new RunCommand(intakePivot::closedLoopIntakePivot, intakePivot))
      .raceWith(new RunCommand(elevatorSubsystem::closedLoopElevator, elevatorSubsystem)),

      new FunctionalCommand(
        // Reset controller on command start
        elevatorSubsystem::resetController,
        // run the elevator to the stow position
        () -> elevatorSubsystem.setHeightStow(),
        // at the end of the command call the closed loop elevator to hold the setpoint
        interrupted -> elevatorSubsystem.closedLoopElevator(),
        // End the command when elevator is at Position
        () -> elevatorSubsystem.isAtHeight(),
        // Require the elevator subsystem
        elevatorSubsystem
      ).raceWith(new RunCommand(crossSlide::closedLoopCrossSlide, crossSlide))
      .raceWith(new RunCommand(intakePivot::closedLoopIntakePivot, intakePivot))








      //new RunCommand(intakePivot::setPositionStow, intakePivot).withTimeout(1.0),
      //new InstantCommand(crossSlide::resetController, crossSlide),
      //.alongWith(new InstantCommand(intakePivot::resetController, intakePivot)),
      //new RunCommand(crossSlide::setPositionStow, crossSlide)
      //.alongWith(new RunCommand(intakePivot::setPositionStow, intakePivot))
      // new RunCommand(crossSlide::setPositionIntake, crossSlide).until(() -> crossSlide.isAtPosition()),

      
    );
  }
}
