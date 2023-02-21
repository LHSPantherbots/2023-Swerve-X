// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class CrossSlideSubsystem extends SubsystemBase {
  
  CANSparkMax crossSlide = new CANSparkMax(RIO_Channels_CAN_MOTOR.CROSS_SLIDE, MotorType.kBrushless);
 
  RelativeEncoder crossSlideEncoder;

  private SparkMaxPIDController crossSlidePidController;
  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double maxVel = 30.0;
  private double maxAcc = 30.0;
  private double allowableError = 1.0;
  private double positionSetpoint = 0.0;
  private double lastSetpoint = 0.0;
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  public BooleanSupplier isAtPos = () -> this.isAtPosition();


  /** Creates a new CrossSlideSubsystem. */
  public CrossSlideSubsystem() {
    crossSlide.restoreFactoryDefaults();

    //Set limit low when starting to keep from destroying itself before tuning;
    crossSlide.setSmartCurrentLimit(15);

    //Adjust this value if the cross slide is accellerating too fast
    crossSlide.setClosedLoopRampRate(0.25);

    crossSlide.setIdleMode(IdleMode.kBrake);

    //Flip these if the cross goes the wrong direction
    crossSlide.setInverted(false);

    crossSlideEncoder = crossSlide.getEncoder();

    crossSlidePidController = crossSlide.getPIDController();

    m_constraints =
      new TrapezoidProfile.Constraints(maxVel, maxAcc);
    m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);


    crossSlide.burnFlash();

  }
  


  @Override
  public void periodic() {
     //Smart Dashboard Items
     SmartDashboard.putNumber("Cross Slide Position", getCrossSlidePosition());
     SmartDashboard.putBoolean("Cross Slide at Set Positon", isAtPosition());
     SmartDashboard.putNumber("Cross Slide Positio Setpoint", getPositionSetpoint());
    
 
 
  }



  public double getCrossSlidePosition(){
    return crossSlideEncoder.getPosition();
  }

  public boolean isAtPosition(){
    double error = getCrossSlidePosition() - positionSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public BooleanSupplier isPos(){
    return () -> this.isAtPosition();
  }

  // public BooleanSupplier supplyIsAtPositoin(){
  //   return isAtPosition();
  // }

  public double getPositionSetpoint(){
    return positionSetpoint;
  }

  public void setPositionSetpoint(double setPoint){
    positionSetpoint = setPoint;
  }

  public void manualCrossSlide(double move){
    crossSlide.set(move);
  }

  public void resetController(){
    m_controller.reset(crossSlideEncoder.getPosition());
  }

  public void stopCrossSlide(){
    crossSlide.set(0.0);
  }

  public void closedLoopCrossSlide(){
    crossSlide.set(m_controller.calculate(crossSlideEncoder.getPosition(), positionSetpoint));
  }


  public void setPositionStow(){
    positionSetpoint = 0.2;
    closedLoopCrossSlide();    
  }

  public void setPositionIntake(){
    positionSetpoint = 2.0;
    closedLoopCrossSlide();
  }

  public void setPositionOut(){
    positionSetpoint = 9.0;
    closedLoopCrossSlide();
  }

}


