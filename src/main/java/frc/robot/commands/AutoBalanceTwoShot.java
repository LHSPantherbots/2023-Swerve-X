// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Position;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceTwoShot extends SequentialCommandGroup {
  double xSpeed;

  ;
  /** Creates a new AutoBalance. */
  public AutoBalanceTwoShot(DriveSubsystem driveSubsystem) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            
              new AutoBalanceSmart(driveSubsystem),
              new RunCommand(driveSubsystem::xWheels, driveSubsystem).withTimeout(3.0),
              new AutoBalanceSmart(driveSubsystem),
              new RunCommand(driveSubsystem::xWheels, driveSubsystem).withTimeout(3.0));

    /* just in case
    new RunCommand(() -> driveSubsystem.drive(this.xSpeed, 0.0, 0.0, true), driveSubsystem)
                .until(() -> driveSubsystem.isAtAutoBalanceAngle()).withTimeout(6.0),
        new RunCommand(driveSubsystem::xWheels, driveSubsystem).withTimeout(3.0), 
        
        new ConditionalCommand(new RunCommand(driveSubsystem::xWheels, driveSubsystem),
        new RunCommand(() -> driveSubsystem.drive(this.xSpeed * -1, 0.0, 0.0, true), driveSubsystem)
        .until(() -> driveSubsystem.isBalanced()),
          () -> driveSubsystem.isBalanced()).withTimeout(6.0));
*/
  }
}
