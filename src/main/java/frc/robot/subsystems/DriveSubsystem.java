// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  //limelight vision may need to be tuned
  private double kP = 0.120;
  private double kF = 0.0; //0.2;

  private Pigeon2  m_gyro = new Pigeon2(DriveConstants.kPigeonCAN_ID);
  
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftAngleZero);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftAngleZero);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightAngleZero);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightAngleZero);


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          getYaw(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
          // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    

    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){
      }
    }).start();
    
  }


  




  @Override
  public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
          getYaw(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });



    //Add values to smartdashborad
    // SmartDashboard.putNumber("Front Left Abs Angle", m_frontLeft.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Front Left Angle", m_frontLeft.getModuleAngle());
    // SmartDashboard.putNumber("Front Right Abs Angle", m_frontRight.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Front Right Angle", m_frontRight.getModuleAngle());
    // SmartDashboard.putNumber("Rear Left Abs Angle", m_rearLeft.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Rear Left Angle", m_rearLeft.getModuleAngle());
    // SmartDashboard.putNumber("Rear Right Abs Angle", m_rearRight.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Rear Right Angle", m_rearRight.getModuleAngle());
    // SmartDashboard.putNumber("Front Left Position", m_frontLeft.getDriveEncoderPosition());
    // SmartDashboard.putNumber("Rear Left Position", m_rearLeft.getDriveEncoderPosition());
    // SmartDashboard.putNumber("Front Right Position", m_frontRight.getDriveEncoderPosition());
    // SmartDashboard.putNumber("Rear Right Position", m_rearRight.getDriveEncoderPosition());
    SmartDashboard.putNumber("Front Left Pos, m", m_frontLeft.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Front Right Pos, m", m_frontRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Back Left Pos, m", m_rearLeft.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Back Right Pos, m", m_rearRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getYaw());

    //SmartDashboard.putNumber("FR Drive Set", m_frontLeft.)
    SmartDashboard.putString("FR Actual State", m_frontRight.getState().toString());

    SmartDashboard.putNumber("Front Right Encoder Distance", m_frontRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Front Right Encoder Speed Meter/s", m_frontRight.getDriveEncoderVelocityMeterPerSec());

    SmartDashboard.putNumber("Front Left Abs Angle", m_frontLeft.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Front Right Abs Angle", m_frontRight.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Rear Left Abs Angle", m_rearLeft.getModuleAbsoluteAngle());
    SmartDashboard.putNumber("Rear Right Abs Angle", m_rearRight.getModuleAbsoluteAngle());


    SmartDashboard.putNumber("Gyro Angle", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Rot 2d", getYaw().toString());






  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }



  
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getYaw(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }




  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    SmartDashboard.putString("FR2 Set State", swerveModuleStates[1].toString());
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    SmartDashboard.putString("FR Set State", desiredStates[1].toString());
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0.0);
  
    
    //m_gyro.reset();
  }

  
  public Rotation2d getYaw(){
    double yawRadians = Math.toRadians(m_gyro.getYaw());
    return new Rotation2d(yawRadians);
  }


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
   public double getHeading() {
    return getYaw().getDegrees();
    
  }
  public void resetAll(){
    resetEncoders();
    zeroHeading();
  }

  public double getHeadingRadians() {
    return getYaw().getRadians();
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */

  //  public double getTurnRate() {
  //  return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  //}

  public void limeLightAim(double xSpeed, double ySpeed)
  {
    double error = RobotContainer.limelight.getHorizontalOffset();
    kF = Math.copySign(kF, error);
    double outF = kF;             
    double outP = kP * error;
    double outputTurn = outF + outP;
    if (RobotContainer.limelight.isTargetValid()){
      drive(xSpeed, ySpeed, -outputTurn, true);
    }else{
      drive(xSpeed, ySpeed, 0.0, true);
    }
  }




}