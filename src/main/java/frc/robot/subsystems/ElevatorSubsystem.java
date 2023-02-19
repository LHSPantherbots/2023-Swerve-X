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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class ElevatorSubsystem extends SubsystemBase {
  
  CANSparkMax elevatorLeader = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_LEADER, MotorType.kBrushless);
  CANSparkMax elevatorFollower = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_FOLLOWER, MotorType.kBrushless);

  RelativeEncoder elevatorEncoder;

  private SparkMaxPIDController elevatorPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowableError, karbFF;
  private double heightSetpoint = 0.0;
  private double lastSetpoint = 0.0;
  private double arbitraryFeedForward = 0.026;
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorLeader.restoreFactoryDefaults();
    elevatorFollower.restoreFactoryDefaults();

    //Set limit low when starting to keep from destroying itself before tuning;
    elevatorLeader.setSmartCurrentLimit(40);
    elevatorFollower.setSmartCurrentLimit(40);

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
    karbFF = 0.5;

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

    SmartDashboard.putNumber("Elevator P Gain", kP);
    SmartDashboard.putNumber("Elevator I Gain", kI);
    SmartDashboard.putNumber("Elevator D Gain", kD);
    SmartDashboard.putNumber("Elevator I Zone", kIz);
    SmartDashboard.putNumber("Elevator Feed Forward", kFF);
    SmartDashboard.putNumber("Elevator Max Output", kMaxOutput);
    SmartDashboard.putNumber("Elevator Min Output", kMinOutput);
    SmartDashboard.putNumber("Elevator Arbitrary Feed Forward", karbFF);  

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Elevator Max Velocity", maxVel);
    SmartDashboard.putNumber("Elevator Min Velocity", minVel);
    SmartDashboard.putNumber("Elevator Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Elevator Allowed Closed Loop Error", allowableError);
    SmartDashboard.putNumber("Elevator Set Position", 0);
    SmartDashboard.putNumber("Elevator Set Velocity", 0);


    m_constraints =
    new TrapezoidProfile.Constraints(30, 30
    );
    m_controller =
    new ProfiledPIDController(0.05, 0.0, 0.0, m_constraints, kDt);
    elevatorLeader.burnFlash();
    elevatorFollower.burnFlash();

  }

  @Override
  public void periodic() {
     //Smart Dashboard Items
     SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
     SmartDashboard.putBoolean("Elevator at Set Height", isAtHeight());
     SmartDashboard.putNumber("Elevator Height Setpoint", getHeightSetpoint());
     SmartDashboard.putNumber("Elevator Feed Forward", arbitraryFeedForward);  
     SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());    
     SmartDashboard.putNumber("Elvator Amps", elevatorLeader.getOutputCurrent());
 
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
  public void reset(){
    m_controller.reset(elevatorEncoder.getPosition());
  }

  public void stopElevator(){
    elevatorLeader.set(0.0);
  }
  
  public void closedLoopElevator(){
    elevatorLeader.set(arbitraryFeedForward + m_controller.calculate(elevatorEncoder.getPosition(), heightSetpoint));
    //elevatorLeader.set(arbitraryFeedForward);
  }

  public void closedLoopElevator2(){
    final double p = SmartDashboard.getNumber("Elevator P Gain", 0);
    final double i = SmartDashboard.getNumber("Elevator I Gain", 0);
    final double d = SmartDashboard.getNumber("Elevator D Gain", 0);
    final double iz = SmartDashboard.getNumber("Elevator I Zone", 0);
    final double ff = SmartDashboard.getNumber("Elevator Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Elevator Max Output", 0);
    final double min = SmartDashboard.getNumber("Elevator Min Output", 0);
    final double arbFF = SmartDashboard.getNumber("Elevator Arbitrary Feed Forward", 0);
    final double maxV = SmartDashboard.getNumber("Elevator Max Velocity", 0);
    final double minV = SmartDashboard.getNumber("Elevator Min Velocity", 0);
    final double maxA = SmartDashboard.getNumber("Elevator Max Acceleration", 0);
    final double allE = SmartDashboard.getNumber("Elevator Allowed Closed Loop Error", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevatorPidController.setP(p); kP = p; }
    if((i != kI)) { elevatorPidController.setI(i); kI = i; }
    if((d != kD)) { elevatorPidController.setD(d); kD = d; }
    if((iz != kIz)) { elevatorPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevatorPidController.setFF(ff); kFF = ff; }
    if((arbFF !=karbFF)) {karbFF = arbFF;}
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        elevatorPidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; }
    if((maxV != maxVel)) { elevatorPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevatorPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevatorPidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowableError)) {elevatorPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowableError = allE; }

    
    elevatorPidController.setReference(heightSetpoint, CANSparkMax.ControlType.kPosition, 0 ,arbFF,ArbFFUnits.kVoltage);
  }


  public void setHeightMid(){
    heightSetpoint = 22.0;
    closedLoopElevator();    
  }

  public void setHeightLow(){
    heightSetpoint = 7.0;
    closedLoopElevator();
  }

  

}


