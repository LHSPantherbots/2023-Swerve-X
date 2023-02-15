// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final boolean invertGyro = false;
        //public static final int kIntakeTalonPort = 30;
        public static final int kPigeonCAN_ID = 30;

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kRearLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kRearRightDriveMotorPort = 4;
    
        public static final int kFrontLeftTurningMotorPort = 21;
        public static final int kRearLeftTurningMotorPort = 22;
        public static final int kFrontRightTurningMotorPort = 23;
        public static final int kRearRightTurningMotorPort = 24;
    
        public static final int kFrontLeftTurningEncoderPort = 0; //Analog input ports on rio
        public static final int kRearLeftTurningEncoderPort = 1;
        public static final int kFrontRightTurningEncoderPort = 2;
        public static final int kRearRightTurningEncoderPort = 3;
    
        public static final double kFrontLeftAngleZero = 0.47; //Set after aligign all wheels forward
        public static final double kRearLeftAngleZero = -116.6;
        public static final double kFrontRightAngleZero = -128.13;
        public static final double kRearRightAngleZero = 177.98;
    
    
        public static final double kTrackWidth = 20.255 * 0.0254; //converts 18.5 inches to meters
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 25.25 * 0.0254; //converts 18.5 inches to meters
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right
    
        public static final boolean kGyroReversed = false;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.0978;
        public static final double kvVoltSecondsPerMeter = 3.16;
        public static final double kaVoltSecondsSquaredPerMeter = 0.274;
    
        public static final double kMaxSpeedMetersPerSecond = 4.4;
      }
    
      public static final class ModuleConstants {
        public static final double kPTurning = 0.5;
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20* 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20* 2 * Math.PI;
    
        public static final double kEncoderCPR = 1.0; //default unit for spark max distance is 1 revolution
        public static final double driveGearReduction = 24.0/12.0* 24.0/22.0 * 45.0/15.0; //Swerve X Ratio
        public static final double kWheelDiameterMeters = 4 * .0254; //4" wheel
        public static final double kDriveEncoderDistancePerPulse =
            (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR *driveGearReduction); //converts motor rpm to meters wheel traveled
    
        public static final double kTurningEncoderDistancePerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) kEncoderCPR;
    
        public static final double kPModuleTurningController = 0.02;//1
    
        public static final double kPModuleDriveController = 0.2;//1
      
        
        public static final double kTurningEncoderRot2Rad = (12.0/24.0)*(14.0/72.0)* 2 * Math.PI;
        
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    
    }

    
    
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class GamePadButtons{
        // Basic buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RB = 6;
        public static final int LB = 5;
        public static final int Select = 7;
        public static final int Start = 8;
        public static final int LeftJ = 9;
        public static final int RightJ = 10;

        //POV buttons
        public static final int Up = 0;  
        public static final int Down = 180;   
        public static final int Left = 270;
        public static final int Right = 90; 

        //Axis
        public static final int leftY = 1;
        public static final int leftX = 0;
        public static final int rightY = 5;
        public static final int rightX = 4;
        public static final int leftTrigger = 2;
        public static final int rightTrigger = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    
    public static final class ElevatorConstants {
        public static final int leftElavatorMotor_ID = 31;
        public static final int rightElavatorMotor_ID = 32;
        public static final double kP = 0.0000035;
        public static final double kI = 0.0;
        public static final double kD = 0.00002; 
        public static final double kIz = 0;
        public static final double kFF = 0.000165;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5700;
        public static final double maxVel = 5700;
        public static final double minVel = -5700;
        public static final double maxAcc = 4000;
        public static final double allowedErr = 0;
    }


}
