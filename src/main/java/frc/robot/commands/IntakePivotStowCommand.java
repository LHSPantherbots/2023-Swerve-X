// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakePivotStowCommand extends CommandBase {
  /** Creates a new IntakePivotStowCommand. */
  private final IntakePivotSubsystem m_intakePivot;
  public IntakePivotStowCommand(IntakePivotSubsystem intakePivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakePivot = intakePivot;
    addRequirements(m_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakePivot.resetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakePivot.setPositionStow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakePivot.closedLoopIntakePivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakePivot.isAtPosition();
  }
}
