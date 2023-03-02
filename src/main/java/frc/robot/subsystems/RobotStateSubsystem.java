// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateSubsystem extends SubsystemBase {
  /** Creates a new RobotStateSubsystem. */
  private boolean coneMode = true; // defaults to cone mode

  public RobotStateSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Cone Mode", getConeMode());
  }

  public boolean getConeMode() {
    return coneMode;
  }

  public void setConeMode(boolean mode) {
    coneMode = mode;
  }
}
