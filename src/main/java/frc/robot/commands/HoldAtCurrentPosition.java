// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldAtCurrentPosition extends ParallelCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public HoldAtCurrentPosition(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setHeightSetpoint(elevator.getElevatorHeight())), //Sets setpoint to current height
        new FunctionalCommand(
          // Reset controller on command start
          elevator::resetController,
          // Turns on closed loop elevator to hold current position
          () -> elevator.closedLoopElevator(),
          // if interruped stops motors
          interrupted -> elevator.stopElevator(),
          // Does not end until interupted
          () -> false,
          // Require the elevator subsystem
          elevator
        )
      ),
      new SequentialCommandGroup(

        new InstantCommand(() -> crossSlide.setPositionSetpoint(crossSlide.getPositionSetpoint())), //Sets current position of cross slide
        new FunctionalCommand(
          // Reset controller on command start
          crossSlide::resetController,
          // holds current positoin of crosslide
          () -> crossSlide.closedLoopCrossSlide(),
          // at the end of the command call stops motors
          interrupted -> crossSlide.stopCrossSlide(),
          // Does not end until interupted
          () -> false,
          // Require the crossSlide subsystem
          crossSlide
        )
      ),
      new SequentialCommandGroup(
        

        new InstantCommand(() -> intakePivot.setPositionSetpoint(intakePivot.getPositionSetpoint())),
        new FunctionalCommand(
          // Reset controller on command start
          intakePivot::resetController,
          // Holds position
          () -> intakePivot.closedLoopIntakePivot(),
          // at the end of the command call stops motors
          interrupted -> intakePivot.intakePivotStop(),
          // Does not end until interupted
          () -> false,
          // Require the intakePivot subsystem
          intakePivot
        )
      )   
    );
  }
}
