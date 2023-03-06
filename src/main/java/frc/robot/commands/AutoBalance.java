// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends SequentialCommandGroup {
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSubsystem driveSubsystem, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
          new RunCommand(() -> driveSubsystem.drive(0.6, 0.0, 0.0, true), driveSubsystem)
              .until(() -> driveSubsystem.isAtAutoBalanceAngle()),
          new ElevatorCmd(Position.HOLD, elevator, false).withTimeout(6.0)
          // .withTimeout(5)
        ),
        new RunCommand(driveSubsystem::xWheels, driveSubsystem));
  }
}
