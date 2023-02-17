// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  public boolean openLoopDrive = false;
  // private final RelativeEncoder turn_Encoder;
  private final AnalogEncoder turn_Encoder;
  private SparkMaxPIDController m_drivePidController, m_turnPidController;
  private final Double offsetAngle;

  public double kP,
      kI,
      kD,
      kIz,
      kFF,
      kMaxOutput,
      kMinOutput,
      maxRPM,
      maxVel,
      minVel,
      maxAcc,
      allowedErr;
  public double kP_turn,
      kI_turn,
      kD_turn,
      kIz_turn,
      kFF_turn,
      kMaxOutput_turn,
      kMinOutput_turn,
      maxRPM_turn,
      maxVel_turn,
      minVel_turn,
      maxAcc_turn,
      allowedErr_turn;

  // private final CANCoder m_turningEncoder;
  // private final AnalogInput m_turningEncoder;

  private final PIDController turningPidController;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the driv e motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningEncoderPort ID for the turning encoder port
   * @param angleZero Absolute angle when module is point forward
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPort,
      double angleZero) { // , // This is the absolute angle of the module pointing forward 0deg

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setInverted(true);

    // initialze PID controller and encoder objects
    m_drivePidController = m_driveMotor.getPIDController();
    this.m_driveEncoder = m_driveMotor.getEncoder();
    this.turn_Encoder = new AnalogEncoder(turningEncoderPort);

    this.offsetAngle = angleZero;

    // this.turn_Encoder.setDistancePerRotation(ModuleConstants.kTurningEncoderRot2Rad);
    // this.turn_Encoder.setPositionOffset(angleZero/360);

    // PID coefficients
    kP = 0.0002; // 5e-5;
    kI = 0.0; // 1e-6;
    kD = 0;
    kIz = 0;
    kFF = 0.000156; // 0.000156;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 5700; // rpm
    maxAcc = 3000;

    // set PID coefficients
    m_drivePidController.setP(kP);
    m_drivePidController.setI(kI);
    m_drivePidController.setD(kD);
    m_drivePidController.setIZone(kIz);
    m_drivePidController.setFF(kFF);
    m_drivePidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    m_drivePidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_drivePidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_drivePidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_drivePidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    kP_turn = 0.0008; // 5e-5;
    kI_turn = 0.0; // 1e-6;
    kD_turn = 0;
    kIz_turn = 0;
    kFF_turn = 0.000156; // 0.000156;
    kMaxOutput_turn = 1;
    kMinOutput_turn = -1;
    maxRPM_turn = 5700;

    // Smart Motion Coefficients
    maxVel_turn = 5700; // rpm
    maxAcc_turn = 3000;

    allowedErr = .1;

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocityMeterPerSec(), new Rotation2d(getModuleAngleRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderPositionMeter(), new Rotation2d(getModuleAngleRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.)
   */
  public void setDesiredState(SwerveModuleState state) {
    // keep modules at current state when no input is given
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, new Rotation2d(getModuleAngleRadians()));

    // double motorRpm = 0;
    double motorRpm = getMotorRpmFromDriveVelocity(state.speedMetersPerSecond);

    if (Math.abs(state.speedMetersPerSecond) > 0.1 * DriveConstants.kMaxSpeedMetersPerSecond) {
      if (openLoopDrive) {
        m_driveMotor.set(state.speedMetersPerSecond);
      } else {
        m_drivePidController.setReference(motorRpm, CANSparkMax.ControlType.kSmartVelocity);
      }
    } else {
      m_drivePidController.setReference(0, CANSparkMax.ControlType.kSmartVelocity); // adds deadband
    }

    m_turningMotor.set(
        turningPidController.calculate(getModuleAngleRadians(), state.angle.getRadians()));
  }

  public void manualDrive(double drive, double turn) {
    m_driveMotor.set(drive);
    m_turningMotor.set(turn);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    turn_Encoder.reset();
  }

  public double getDriveEncoderPosition() { // devault unit revolution
    return m_driveEncoder.getPosition();
  }

  public double getDriveEncoderVelocity() { // defualt unit is rpm
    return m_driveEncoder.getVelocity();
  }

  public double getModuleAbsoluteAngle() {

    return turn_Encoder.getAbsolutePosition();
  }

  public double getDriveEncoderPositionMeter() {
    return m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderDistancePerPulse;
  }

  public double getMotorRpmFromDriveVelocity(double velocity) {

    return velocity * 60 / ModuleConstants.kDriveEncoderDistancePerPulse;
  }

  public double getDriveEncoderVelocityMeterPerSec() {
    return getDriveEncoderVelocity()
        * ModuleConstants.kDriveEncoderDistancePerPulse
        / 60; // converts rpm to
    // meters/second
  }

  public double getModuleAngle() {

    // modulo 360 keeps noise from making the provided value go above 360
    double rawAngle = (turn_Encoder.getAbsolutePosition() * 360 - this.offsetAngle) % 360;
    double angle;
    if (rawAngle > 180.0 && rawAngle < 360.0) {
      angle = -180 + rawAngle % 180.0;
    } else {
      angle = rawAngle;
    }
    return angle;
  }

  public double getModuleAngleRadians() {

    return getModuleAngle() * Math.PI / 180.0;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
}
