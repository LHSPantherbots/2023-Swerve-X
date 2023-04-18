// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  DriveSubsystem m_DriveSubsystem;

  XboxController m_controller;
  double startAngle;
  boolean atWall = false;
  double reducedspeedconstant = 0.2;

  public DriveStraight(DriveSubsystem driveSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = driveSubsystem;
    m_controller = controller;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //startAngle = m_DriveSubsystem.getHeading(); uses current angle
    startAngle = 0.0; //makes robot turn to 0 angle field relative
    m_controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.crossSlide.getCrossSlideCurrent() > 5) {
      m_controller.setRumble(RumbleType.kBothRumble, 1.0);
      m_DriveSubsystem.driveStraight(0.0, 0.0, startAngle);
    }
    else{
      m_controller.setRumble(RumbleType.kBothRumble, 0.0);
      m_DriveSubsystem.driveStraight(0.6, 
                                    -m_controller.getLeftX() * reducedspeedconstant
                                    * DriveConstants.kMaxSpeedMetersPerSecond, 
                                    startAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0.0, 0.0, 0.0, true);
    m_controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
