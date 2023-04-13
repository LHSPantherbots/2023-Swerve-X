// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class IntakePivotSubsystem extends SubsystemBase {

  CANSparkMax intakePivot =
      new CANSparkMax(RIO_Channels_CAN_MOTOR.INTAKE_PIVOT, MotorType.kBrushless);

  CANCoder intakePivotAbsoluteEncoder =
      new CANCoder(RIO_Channels_CAN_MOTOR.INTAKE_PIVOT_ABS_INCODER);

  RelativeEncoder intakePivotEncoder;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private static double kDt = 0.02;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  private double kP = 0.015; // 0.02
  private double kI = 0.0;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double maxVel = 500.0; // 100.0; //deg/sec
  private double maxAcc = 600.0; // 250.0; //deg/sec/sed
  private double karbFF = 0.014; // Scaling Constant for arbitrary feed forward.
  private double allowableError = 2.0;
  private double positionSetpoint = 180.0;
  private double lastSetpoint = 0.0;

  private double armPivotRatio = 4 * 4 * 4 * 38 / 16;

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    intakePivot.restoreFactoryDefaults();

    // Set limit low when starting to keep from destroying itself before tuning;
    intakePivot.setSmartCurrentLimit(40);

    // Adjust this value if the intake pivot is accellerating too fast
    intakePivot.setClosedLoopRampRate(0.25);

    intakePivot.setIdleMode(IdleMode.kBrake);

    // Flip these if the intake pivot goes the wrong direction
    intakePivot.setInverted(true);

    intakePivotEncoder = intakePivot.getEncoder();

    intakePivotAbsoluteEncoder.configMagnetOffset(
        30.0); // spare intake 149 straight up is approxamatly where 180 is on current sensor

    m_constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);
    m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    intakePivot.burnFlash();
  }

  @Override
  public void periodic() {
    // Smart Dashboard Items
    SmartDashboard.putNumber("Intake Pivot Position", getintakePivotPosition());
    SmartDashboard.putBoolean("Intake Pivot at Set Positon", isAtPosition());
    SmartDashboard.putNumber("Intake Pivot Position Setpoint", getPositionSetpoint());
    SmartDashboard.putNumber(
        "Intake Pivot Absolute Encoder Position", intakePivotAbsoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber(
        "Intake Pivot Absolute Encoder Velocity", intakePivotAbsoluteEncoder.getVelocity());
    SmartDashboard.putNumber(
        "Intake Pivot Feed Forward Voltage Output", calculateArbitraryFeedforward());
    SmartDashboard.putNumber("Intake Motor Voltage", intakePivot.getAppliedOutput());
    SmartDashboard.putNumber("Intake Motor Current", intakePivot.getOutputCurrent());
  }

  // Currently not used but keeping this incase it is needed
  public double calculateArbitraryFeedforward() {
    double output =
        karbFF * Math.sin(Math.toRadians(intakePivotAbsoluteEncoder.getAbsolutePosition()));
    return output;
  }

  public double getintakePivotPosition() {
    return intakePivotAbsoluteEncoder.getAbsolutePosition();
  }

  public boolean isAtPosition() {
    double error = getintakePivotPosition() - positionSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public void setPositionSetpoint(double setPoint) {
    positionSetpoint = setPoint;
  }

  public void manualintakePivot(double move) {
    intakePivot.set(move + calculateArbitraryFeedforward());
  }

  public void intakePivotStop() {
    intakePivot.set(0.0);
  }

  public void resetController() {
    m_controller.reset(intakePivotAbsoluteEncoder.getAbsolutePosition());
  }

  public void closedLoopIntakePivot() {

    intakePivot.set(
        calculateArbitraryFeedforward()
            + m_controller.calculate(
                intakePivotAbsoluteEncoder.getAbsolutePosition(), positionSetpoint));
  }

  public void setPositionStow() {
    positionSetpoint = 180.0;
    closedLoopIntakePivot();
  }

  public void setPositionintakeCone() {
    positionSetpoint = 82.0;
    closedLoopIntakePivot();
  }

  public void setPositionintakeCube() {
    // positionSetpoint = 86.0;
    positionSetpoint = 78.0; // 80.0; // 75
    closedLoopIntakePivot();
  }

  public void setPositionScoreCone() {
    positionSetpoint = 82.0;
    closedLoopIntakePivot();
  }

  public void setLevelt3ConeScore() {
    positionSetpoint = 110.0;
    closedLoopIntakePivot();
  }

  public void setLevelt2ConeScore() {
    positionSetpoint = 136.5;
    closedLoopIntakePivot();
  }

  public void setLevelt3CubeScore() {
    positionSetpoint = 126.0;
    closedLoopIntakePivot();
  }

  public void setLevelt2CubeScore() {
    positionSetpoint = 123.0;
    closedLoopIntakePivot();
  }

  public void setLevelCobraIntake() {
    positionSetpoint = 165.0;
    closedLoopIntakePivot();
  }

  public void setPositonIntakeConeDoubleSubstation() {
    positionSetpoint = 106.8; // may need to ajust//106.8
    closedLoopIntakePivot();
  }
}
