// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class ElevatorSubsystem extends SubsystemBase {
  
  CANSparkMax elevatorLeader = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_LEADER, MotorType.kBrushless);
  CANSparkMax elevatorFollower = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_FOLLOWER, MotorType.kBrushless);

  RelativeEncoder elevatorEncoder;

  private SparkMaxPIDController elevatorPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowableError, arbFF;
  private double heightSetpoint = 0.0;
  private double lastSetpoint = 0.0;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();

    //Set limit low when starting to keep from destroying itself before tuning;
    elevatorLeader.setSmartCurrentLimit(15);
    elevatorFollower.setSmartCurrentLimit(15);

    //Adjust this value if the elevator is accellerating too fast
    elevatorLeader.setClosedLoopRampRate(0.25);
    elevatorFollower.setClosedLoopRampRate(0.25);

    elevatorLeader.setIdleMode(IdleMode.kBrake);
    elevatorFollower.setIdleMode(IdleMode.kBrake);

    //Flip these if the elevator goes the wrong direction
    elevatorLeader.setInverted(false);

    //Sets up follower to mimic the leader
    elevatorFollower.follow(elevatorLeader, true);

    elevatorEncoder = elevatorLeader.getEncoder();

    elevatorPidController = elevatorLeader.getPIDController();

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
    arbFF = 0.5;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;


     // set PID coefficients
    elevatorPidController.setP(kP);
    elevatorPidController.setI(kI);
    elevatorPidController.setD(kD);
    elevatorPidController.setIZone(kIz);
    elevatorPidController.setFF(kFF);
    elevatorPidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    elevatorPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    elevatorPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    elevatorPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    elevatorPidController.setSmartMotionAllowedClosedLoopError(allowableError, smartMotionSlot);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowableError);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    elevatorLeader.burnFlash();
    elevatorFollower.burnFlash();

  }

  @Override
  public void periodic() {
     //Smart Dashboard Items
     SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
     SmartDashboard.putBoolean("At Set Height", isAtHeight());
     SmartDashboard.putNumber("Height Setpoing", getHeightSetpoint());
    
 
 
  }



  public double getElevatorHeight(){
    return elevatorEncoder.getPosition();
  }

  public boolean isAtHeight(){
    double error = getElevatorHeight() - heightSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public double getHeightSetpoint(){
    return heightSetpoint;
  }

  public void setHeightSetpoint(double setPoint){
    heightSetpoint = setPoint;
  }

  public void manualElevator(double move){
    elevatorLeader.set(move);
  }

  public void closedLoopElevator(){
    final double p = SmartDashboard.getNumber("P Gain", 0);
    final double i = SmartDashboard.getNumber("I Gain", 0);
    final double d = SmartDashboard.getNumber("D Gain", 0);
    final double iz = SmartDashboard.getNumber("I Zone", 0);
    final double ff = SmartDashboard.getNumber("Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Max Output", 0);
    final double min = SmartDashboard.getNumber("Min Output", 0);
    final double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    final double minV = SmartDashboard.getNumber("Min Velocity", 0);
    final double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    final double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevatorPidController.setP(p); kP = p; }
    if((i != kI)) { elevatorPidController.setI(i); kI = i; }
    if((d != kD)) { elevatorPidController.setD(d); kD = d; }
    if((iz != kIz)) { elevatorPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevatorPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        elevatorPidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; }
    if((maxV != maxVel)) { elevatorPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevatorPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevatorPidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowableError)) {elevatorPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowableError = allE; }

    
    elevatorPidController.setReference(heightSetpoint, CANSparkMax.ControlType.kPosition, 0 ,arbFF,ArbFFUnits.kVoltage);
  }

}


