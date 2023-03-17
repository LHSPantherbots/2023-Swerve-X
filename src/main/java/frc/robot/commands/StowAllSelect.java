// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowAllSelect extends SequentialCommandGroup {
  /** Creates a new AutoStowSelect. */
  public StowAllSelect(
      CrossSlideSubsystem crossSlide,
      IntakePivotSubsystem intakePivot,
      ElevatorSubsystem elevator) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            new StowAllQuick(crossSlide, intakePivot, elevator),
            new StowAll(crossSlide, intakePivot, elevator),
            () -> RobotContainer.robotState.isQuickStowSafe()));
  }
}
