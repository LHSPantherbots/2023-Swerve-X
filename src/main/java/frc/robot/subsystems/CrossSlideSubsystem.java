// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class CrossSlideSubsystem extends SubsystemBase {
  
  CANSparkMax crossSlide = new CANSparkMax(RIO_Channels_CAN_MOTOR.CROSS_SLIDE, MotorType.kBrushless);
 
  RelativeEncoder crossSlideEncoder;

  private SparkMaxPIDController crossSlidePidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowableError;
  private double positionSetpoint = 0.0;
  private double lastSetpoint = 0.0;


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

    // PID coefficients these will need to be tuned
    kP = 0.00015; 
    kI =  0;
    kD = 0.0008; 
    kIz = 0;
    kFF = 0.000;
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    allowableError = 50;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;


     // set PID coefficients
    crossSlidePidController.setP(kP);
    crossSlidePidController.setI(kI);
    crossSlidePidController.setD(kD);
    crossSlidePidController.setIZone(kIz);
    crossSlidePidController.setFF(kFF);
    crossSlidePidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    crossSlidePidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    crossSlidePidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    crossSlidePidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    crossSlidePidController.setSmartMotionAllowedClosedLoopError(allowableError, smartMotionSlot);

    SmartDashboard.putNumber("Cross Slide P Gain", kP);
    SmartDashboard.putNumber("Cross Slide I Gain", kI);
    SmartDashboard.putNumber("Cross Slide D Gain", kD);
    SmartDashboard.putNumber("Cross Slide I Zone", kIz);
    SmartDashboard.putNumber("Cross Slide Feed Forward", kFF);
    SmartDashboard.putNumber("Cross Slide Max Output", kMaxOutput);
    SmartDashboard.putNumber("Cross Slide Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Cross Slide Max Velocity", maxVel);
    SmartDashboard.putNumber("Cross Slide Min Velocity", minVel);
    SmartDashboard.putNumber("Cross Slide Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Cross Slide Allowed Closed Loop Error", allowableError);
    SmartDashboard.putNumber("Cross Slide Set Position", 0);
    SmartDashboard.putNumber("Cross Slide Set Velocity", 0);

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

  public double getPositionSetpoint(){
    return positionSetpoint;
  }

  public void setPositionSetpoint(double setPoint){
    positionSetpoint = setPoint;
  }

  public void manualCrossSlide(double move){
    crossSlide.set(move);
  }

  public void stopCrossSlide(){
    crossSlide.set(0.0);
  }

  public void closedLoopCrossSlide(){
    final double p = SmartDashboard.getNumber("Cross Slide P Gain", 0);
    final double i = SmartDashboard.getNumber("Cross Slide I Gain", 0);
    final double d = SmartDashboard.getNumber("Cross Slide D Gain", 0);
    final double iz = SmartDashboard.getNumber("Cross Slide I Zone", 0);
    final double ff = SmartDashboard.getNumber("Cross Slide Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Cross Slide Max Output", 0);
    final double min = SmartDashboard.getNumber("Cross Slide Min Output", 0);
    final double maxV = SmartDashboard.getNumber("Cross Slide Max Velocity", 0);
    final double minV = SmartDashboard.getNumber("Cross Slide Min Velocity", 0);
    final double maxA = SmartDashboard.getNumber("Cross Slide Max Acceleration", 0);
    final double allE = SmartDashboard.getNumber("Cross Slide Allowed Closed Loop Error", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { crossSlidePidController.setP(p); kP = p; }
    if((i != kI)) { crossSlidePidController.setI(i); kI = i; }
    if((d != kD)) { crossSlidePidController.setD(d); kD = d; }
    if((iz != kIz)) { crossSlidePidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { crossSlidePidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        crossSlidePidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; }
    if((maxV != maxVel)) { crossSlidePidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { crossSlidePidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { crossSlidePidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowableError)) {crossSlidePidController.setSmartMotionAllowedClosedLoopError(allE,0); allowableError = allE; }

    
    crossSlidePidController.setReference(positionSetpoint, CANSparkMax.ControlType.kPosition);
  }


  public void setPositionMid(){
    positionSetpoint = 40;
    closedLoopCrossSlide();    
  }

  public void setPositionIn(){
    positionSetpoint = 10;
    closedLoopCrossSlide();
  }

}


