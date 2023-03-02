// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldAtCurrentPosition extends ParallelCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public HoldAtCurrentPosition(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    elevator.setHeightSetpoint(
                        elevator.getElevatorHeight())), // Sets setpoint to current height
            new ElevatorCmd(Position.HOLD, elevator, false)),
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    crossSlide.setPositionSetpoint(
                        crossSlide
                            .getCrossSlidePosition())), // Sets current position of cross slide
            new CrossSlideCmd(Position.HOLD, crossSlide, false)),
        new SequentialCommandGroup(
            new InstantCommand(
                () -> intakePivot.setPositionSetpoint(intakePivot.getintakePivotPosition())),
            new IntakePivotCmd(Position.HOLD, intakePivot, false)));
  }
}
