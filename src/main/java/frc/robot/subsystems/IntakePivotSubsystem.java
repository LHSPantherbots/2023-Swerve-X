// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class IntakePivotSubsystem extends SubsystemBase {
  
  CANSparkMax intakePivot = new CANSparkMax(RIO_Channels_CAN_MOTOR.INTAKE_PIVOT, MotorType.kBrushless);
 
  CANCoder intakePivotAbsoluteEncoder = new CANCoder(RIO_Channels_CAN_MOTOR.INTAKE_PIVOT_ABS_INCODER);

  RelativeEncoder intakePivotEncoder;

    // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  private SparkMaxPIDController intakePivotPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowableError, karbFF;
  private double positionSetpoint = 0.0;
  private double lastSetpoint = 0.0;
  



  private double armPivotRatio = 4 * 4 * 4 * 38 / 16 ;

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    intakePivot.restoreFactoryDefaults();

    //Set limit low when starting to keep from destroying itself before tuning;
    intakePivot.setSmartCurrentLimit(15);

    //Adjust this value if the intake pivot is accellerating too fast
    intakePivot.setClosedLoopRampRate(0.25);

    intakePivot.setIdleMode(IdleMode.kBrake);

    //Flip these if the intake pivot goes the wrong direction
    intakePivot.setInverted(true);

    intakePivotEncoder = intakePivot.getEncoder();

    intakePivotPidController = intakePivot.getPIDController();

    // PID coefficients these will need to be tuned
    kP = 0.00015; 
    kI =  0;
    kD = 0.0008; 
    kIz = 0;
    kFF = 0.000;
    kMaxOutput = 1; 
    kMinOutput = -1;
    karbFF = 0.0;
    maxRPM = 5700;
    allowableError = 50;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;


     // set PID coefficients
    intakePivotPidController.setP(kP);
    intakePivotPidController.setI(kI);
    intakePivotPidController.setD(kD);
    intakePivotPidController.setIZone(kIz);
    intakePivotPidController.setFF(kFF);
    intakePivotPidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    intakePivotPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    intakePivotPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    intakePivotPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    intakePivotPidController.setSmartMotionAllowedClosedLoopError(allowableError, smartMotionSlot);

    SmartDashboard.putNumber("Intake Pivot P Gain", kP);
    SmartDashboard.putNumber("Intake Pivot I Gain", kI);
    SmartDashboard.putNumber("Intake Pivot D Gain", kD);
    SmartDashboard.putNumber("Intake Pivot I Zone", kIz);
    SmartDashboard.putNumber("Intake Pivot Feed Forward", kFF);
    SmartDashboard.putNumber("Intake Pivot Max Output", kMaxOutput);
    SmartDashboard.putNumber("Intake Pivot Min Output", kMinOutput);
    SmartDashboard.putNumber("Intake Pivot Arbitrary Feed Forward Constant", karbFF);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Intake Pivot Max Velocity", maxVel);
    SmartDashboard.putNumber("Intake Pivot Min Velocity", minVel);
    SmartDashboard.putNumber("Intake Pivot Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Intake Pivot Allowed Closed Loop Error", allowableError);
    SmartDashboard.putNumber("Intake Pivot Set Position", 0);
    SmartDashboard.putNumber("Intake Pivot Set Velocity", 0);

    m_constraints =
      new TrapezoidProfile.Constraints(100, 250);
      m_controller =
      new ProfiledPIDController(0.01, 0.0, 0.0, m_constraints, kDt);

    intakePivot.burnFlash();

  }

  @Override
  public void periodic() {
     //Smart Dashboard Items
     SmartDashboard.putNumber("Intake Pivot Position", getintakePivotPosition());
     SmartDashboard.putBoolean("Intake Pivot at Set Positon", isAtPosition());
     SmartDashboard.putNumber("Intake Pivot Position Setpoint", getPositionSetpoint());
     SmartDashboard.putNumber("Intake Pivot Absolute Encoder Position", intakePivotAbsoluteEncoder.getAbsolutePosition());
     SmartDashboard.putNumber("Intake Pivot Absolute Encoder Velocity", intakePivotAbsoluteEncoder.getVelocity());
     SmartDashboard.putNumber("Intake Pivot Feed Forward Voltage Output", calculateArbitraryFeedforward());
     SmartDashboard.putNumber("Motor Voltage", intakePivot.getAppliedOutput());

  }



  public double calculateArbitraryFeedforward(){
    double output = karbFF * Math.cos(Math.toRadians(intakePivotAbsoluteEncoder.getAbsolutePosition()));
    return output;
  }
  
  public double getintakePivotPosition(){
    return intakePivotEncoder.getPosition();
  }

  public boolean isAtPosition(){
    double error = getintakePivotPosition() - positionSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public double getPositionSetpoint(){
    return positionSetpoint;
  }

  public void setPositionSetpoint(double setPoint){
    positionSetpoint = setPoint;
  }

  public void manualintakePivot(double move){
    intakePivot.set(move);
  }

  public void reset(){
    m_controller.reset(intakePivotAbsoluteEncoder.getAbsolutePosition());
  }

  public void closedLoopIntakePivot(){
    intakePivot.set(m_controller.calculate(intakePivotAbsoluteEncoder.getAbsolutePosition(), positionSetpoint));
  }


  public void closedLoopIntakePivot2(){
    final double p = SmartDashboard.getNumber("Intake Pivot P Gain", 0);
    final double i = SmartDashboard.getNumber("Intake Pivot I Gain", 0);
    final double d = SmartDashboard.getNumber("Intake Pivot D Gain", 0);
    final double iz = SmartDashboard.getNumber("Intake Pivot I Zone", 0);
    final double ff = SmartDashboard.getNumber("Intake Pivot Feed Forward", 0);
    final double arbff = SmartDashboard.getNumber("Intake Pivot Arbitrary Feed Forward", 0);
    
    final double max = SmartDashboard.getNumber("Intake Pivot Max Output", 0);
    final double min = SmartDashboard.getNumber("Intake Pivot Min Output", 0);
    final double maxV = SmartDashboard.getNumber("Intake Pivot Max Velocity", 0);
    final double minV = SmartDashboard.getNumber("Intake Pivot Min Velocity", 0);
    final double maxA = SmartDashboard.getNumber("Intake Pivot Max Acceleration", 0);
    final double allE = SmartDashboard.getNumber("Intake Pivot Allowed Closed Loop Error", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { intakePivotPidController.setP(p); kP = p; }
    if((i != kI)) { intakePivotPidController.setI(i); kI = i; }
    if((d != kD)) { intakePivotPidController.setD(d); kD = d; }
    if((iz != kIz)) { intakePivotPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { intakePivotPidController.setFF(ff); kFF = ff; }
    if((arbff != karbFF)) { karbFF = arbff;}
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        intakePivotPidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; }
    if((maxV != maxVel)) { intakePivotPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { intakePivotPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { intakePivotPidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowableError)) {intakePivotPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowableError = allE; }

    
    intakePivotPidController.setReference(positionSetpoint, CANSparkMax.ControlType.kPosition, 0);
  }


  public void setPositionMid(){
    positionSetpoint = 40;
    closedLoopIntakePivot();    
  }

  public void setPositionIn(){
    positionSetpoint = 10;
    closedLoopIntakePivot();
  }
  
  public void setPositionStow(){
    positionSetpoint = 355;
    closedLoopIntakePivot();
  }

  public void setPositionintakeCone(){
    positionSetpoint = 257;
    closedLoopIntakePivot();
  }

  public void setPositionintakeCube(){
    positionSetpoint = 252;
    closedLoopIntakePivot();
  }


}


