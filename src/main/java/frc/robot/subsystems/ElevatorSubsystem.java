// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class ElevatorSubsystem extends SubsystemBase {
  
  CANSparkMax elevatorLeader = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_LEADER, MotorType.kBrushless);
  CANSparkMax elevatorFollower = new CANSparkMax(RIO_Channels_CAN_MOTOR.ELEVATOR_FOLLOWER, MotorType.kBrushless);

  RelativeEncoder elevatorEncoder;

  private double kP = 0.05;
  private double kI =0.0;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double maxVel = 30.0;
  private double maxAcc = 30.0;
  private double allowableError = 1.0;
  private double heightSetpoint = 0.0;
  private double lastSetpoint = 0.0;
  private double arbitraryFeedForward = 0.026; //duty cycle required to nearly hold up elevator
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

    m_constraints =
      new TrapezoidProfile.Constraints(maxVel, maxAcc);
    m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    
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
    //keeps it from crashing down
    if(move < -0.1){
      move = -0.1;
      }
    elevatorLeader.set(arbitraryFeedForward + move);
  }
  public void resetController(){
    m_controller.reset(elevatorEncoder.getPosition());
  }

  public void stopElevator(){
    elevatorLeader.set(0.0);
  }
  
  public void closedLoopElevator(){
    elevatorLeader.set(arbitraryFeedForward + m_controller.calculate(elevatorEncoder.getPosition(), heightSetpoint));
    //elevatorLeader.set(arbitraryFeedForward);
  }

  public void setHeightHigh(){
    heightSetpoint = 23.0;
    closedLoopElevator();
  }
  
  
  public void setHeightMid(){
    heightSetpoint = 22.0; // about 2/3 up
    closedLoopElevator();    
  }

  public void setHeightLow(){
    heightSetpoint = 7.0; // aobout 1/3 up
    closedLoopElevator();
  }

  public void setHeightStow(){
    heightSetpoint = 0.5; // aobout 1/3 up
    closedLoopElevator();
  }

  public void setLevelt3ConeScore(){
    heightSetpoint = 28.8;
    closedLoopElevator();
  }

  public void setLevelt2ConeScore(){
    heightSetpoint = 18.6;
    closedLoopElevator();
  }

  public void setLevelt2CubeScore(){
    heightSetpoint = 13.4;
    closedLoopElevator();
  }

  public void setLevelt3CubeScore(){
    heightSetpoint = 25.0;
    closedLoopElevator();
  }

  public void setHeightConeIntakeDoubleSubstation(){
    heightSetpoint = 28.38;//may need to adjust have not checked
    closedLoopElevator();
  }

}


