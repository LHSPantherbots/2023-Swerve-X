// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  
  NetworkTable table;
  private boolean validTargets;
  private double horizontalOffset;
  private double verticalOffset;
  private double targetArea;

  public LimeLight() {
    table=NetworkTableInstance.getDefault().getTable("limelight-wilson");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    validTargets = isTargetValid();
    horizontalOffset = getHorizontalOffset();
    verticalOffset = getVerticalOffset();
    targetArea = getTargetArea();

    //Smart Dashboard Items
    SmartDashboard.putNumber("LL Horizontal Offset", horizontalOffset);
    SmartDashboard.putNumber("LL Vertical Offset", verticalOffset);
    SmartDashboard.putNumber("LL Target Area", targetArea);
    SmartDashboard.putBoolean("LL Valid Target", validTargets);
  }

  public boolean isTargetValid(){
    return (table.getEntry("tv").getDouble(0) == 1);
  }

  public double getHorizontalOffset(){
    return table.getEntry("tx").getDouble(0);
  }

  public double getVerticalOffset(){
    return table.getEntry("ty").getDouble(0);
  }

  public double getTargetArea(){
    return table.getEntry("ta").getDouble(0);
  }

  public double getSkew(){
    return table.getEntry("ts").getDouble(0);
  }

  public double getActivePipeline(){
    return table.getEntry("getpipe").getDouble(0);
  }

  //Forces LED's off
  public void ledOff(){
    table.getEntry("ledMode").setNumber(1);
  }

  //forces LED's On
  public void ledOn(){
      table.getEntry("ledMode").setNumber(3);
  }

  //Sets LED's to be in the state of the current pipeline
  public void ledPipeline(){
    table.getEntry("ledMode").setNumber(0);
  }

  public void setPipeline(int pipeline){
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public void setPipelineZero(){
    table.getEntry("pipeline").setNumber(0);
  }

  public void setPipelineOne(){
    table.getEntry("pipeline").setNumber(1);
  }

  public void setPipelineThree(){
    table.getEntry("pipeline").setNumber(3);
  }

  public void startTakingSnapshots(){
    table.getEntry("snapshot").setNumber(1);
  }

  public void stopTakingSnapshots(){
    table.getEntry("snapshot").setNumber(0);
  }
}
