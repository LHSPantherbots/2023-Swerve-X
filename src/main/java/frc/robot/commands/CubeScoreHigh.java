// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.InstanceAlreadyExistsException;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeScoreHigh extends SequentialCommandGroup {
  /** Creates a new IntakeCubeCommand. */
  public CubeScoreHigh(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new ParallelDeadlineGroup(
        new FunctionalCommand(
          // Reset controller on command start
          elevatorSubsystem::resetController,
          // Start moving intake to high position
          () -> elevatorSubsystem.setLevelt3CubeScore(),
          // at the end of the command call the closed loop elevator to hold the setpoint position
          interrupted -> elevatorSubsystem.setLevelt3CubeScore(),
          // End the command when the elevator is at position
          () -> elevatorSubsystem.isAtHeight(),
          // Require the elevator subsystem
          elevatorSubsystem
        //Added these race withs (test to see if they work)
        )
        .raceWith(new InstantCommand(intakePivot::resetController, intakePivot)
        .andThen(new RunCommand(intakePivot::closedLoopIntakePivot, intakePivot))
        ),

        new RunCommand(crossSlide::closedLoopCrossSlide,crossSlide)
        .withTimeout(0.75)
        .andThen(
          new FunctionalCommand(
            // Reset controller on command start
            crossSlide::resetController,
            // run the crossSlide to the out position
            () -> crossSlide.setLevelt3CubeScore(),
            // at the end of the command call the closed loop cross slide to hold the setpoint
            interrupted -> crossSlide.setLevelt3CubeScore(),
            // End the command when intakePivot is at Position
            () -> crossSlide.isAtPosition(),
            // Require the crossSlide subsystem
            crossSlide
          )
        )
      ),

       new FunctionalCommand(
        // Reset controller on command start
        intakePivot::resetController,
        // Start movint intake pivot to score position
        () -> intakePivot.setLevelt3CubeScore(),
        // at the end of the command call the closed loop intake to hold the setpoint position
        interrupted -> intakePivot.setLevelt3CubeScore(),
        // End the command when the intakePivot is at position
        () -> intakePivot.isAtPosition(),
         // Require the intakePivot subsystem
         intakePivot
        )
        .raceWith(new RunCommand(elevatorSubsystem::closedLoopElevator, elevatorSubsystem))
        .raceWith(new RunCommand(crossSlide::closedLoopCrossSlide, crossSlide))
    );
  }
}
