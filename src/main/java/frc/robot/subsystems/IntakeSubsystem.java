// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RIO_Channels_CAN_MOTOR;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intake = new CANSparkMax(RIO_Channels_CAN_MOTOR.INTAKE, MotorType.kBrushless);

  private final PIDController m_controller;

  RelativeEncoder intakeEncoder;

  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double allowableError = 0.5;
  private double positionSetpoint = 0.0;

  /** Creates a new IntakePivotSubsystem. */
  public IntakeSubsystem() {
    intake.restoreFactoryDefaults();

    // Set limit low when starting to keep from destroying itself before tuning;
    intake.setSmartCurrentLimit(30);

    intake.setIdleMode(IdleMode.kBrake);

    // Flip these if the intake goes the wrong direction
    intake.setInverted(false);
    this.intakeEncoder = intake.getEncoder();

    m_controller = new PIDController(kP, kI, kD);

    intake.burnFlash();
  }

  @Override
  public void periodic() {
    // Smart Dashboard Items
    // SmartDashboard.putNumber("Intake Pivot Position", getintakePivotPosition());
    SmartDashboard.putNumber("Intake Amps", intake.getOutputCurrent());
  }

  public void resetEncoder() {
    intakeEncoder.setPosition(0.0);
  }

  public void manualintake(double move) {
    intake.set(move);
  }

  public void closedLoopIntake() {
    intake.set(m_controller.calculate(intakeEncoder.getPosition(), positionSetpoint));
  }

  public void intakeCube() {
    intake.set(0.60);
  }

  public void ejectCube() {
    intake.set(-.3);
  }

  public void intakeCone() {
    intake.set(-0.8);
  }

  public void ejectCone() {
    intake.set(0.3);
  }

  public void stopIntake() {
    intake.set(0.0);
  }
}
