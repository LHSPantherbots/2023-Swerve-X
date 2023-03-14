// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class ConeIntakeDoubleSubstation extends SequentialCommandGroup {
  /** Creates a new IntakeConeCommand. */
  public ConeIntakeDoubleSubstation(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        // new ParallelRaceGroup(
        //    new ElevatorCmd(Position.CONE_STATION_INTAKE, elevatorSubsystem),
        //    new CrossSlideCmd(Position.CONE_STATION_INTAKE, crossSlide, false),
        //    new IntakePivotCmd(Position.STOW, intakePivot, false)),
        new ParallelRaceGroup(
            new IntakePivotCmd(Position.CONE_STATION_INTAKE, intakePivot, false),
            new ElevatorCmd(Position.CONE_STATION_INTAKE, elevatorSubsystem, true),
            new CrossSlideCmd(Position.CONE_STATION_INTAKE, crossSlide, false)));

    //RobotContainer.robotState.setPosition(Position.CONE_STATION_INTAKE);
  }
}
