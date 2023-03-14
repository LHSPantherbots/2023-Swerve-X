// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeIntakeGround extends SequentialCommandGroup {
  /** Creates a new IntakeCubeCommand. */
  public CubeIntakeGround(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            // if elevator is up
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new IntakePivotCmd(Position.STOW, intakePivot),
                    new CrossSlideCmd(Position.STOW, crossSlide, false),
                    new ElevatorCmd(Position.HOLD, elevator, false)),
                new ParallelRaceGroup(
                    new ElevatorCmd(Position.CUBE_INTAKE, elevator),
                    new CrossSlideCmd(Position.CUBE_INTAKE, crossSlide, false),
                    new IntakePivotCmd(Position.STOW, intakePivot, false)),
                new ParallelRaceGroup(
                    new IntakePivotCmd(Position.CUBE_INTAKE, intakePivot),
                    new ElevatorCmd(Position.CUBE_INTAKE, elevator, false),
                    new CrossSlideCmd(Position.CUBE_INTAKE, crossSlide, false))),
            // If elevator is down

            new ParallelRaceGroup(
                new IntakePivotCmd(Position.CUBE_INTAKE, intakePivot),
                new ElevatorCmd(Position.CUBE_INTAKE, elevator, false),
                new CrossSlideCmd(Position.CUBE_INTAKE, crossSlide, false)),

            // checks elevator position
            () -> (elevator.getElevatorHeight() > 10.0)));
    // RobotContainer.robotState.setPosition(Position.CUBE_INTAKE);
  }
}
