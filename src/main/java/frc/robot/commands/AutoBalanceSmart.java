// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceSmart extends CommandBase {
  /** Creates a new AutoBalance3. */

  DriveSubsystem m_driveSubsystem;
  double driveDirection;
  public AutoBalanceSmart(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_driveSubsystem.getRoll() > 0){
      driveDirection = -1.0;
    }
    else{
      driveDirection = 1.0;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_driveSubsystem.drive(driveDirection * 0.6, 0, 0, false);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return m_driveSubsystem.isBalanced();

  }
}
