package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    CANSparkMax liftMotor = new CANSparkMax(ElevatorConstants.leftElavatorMotor_ID, MotorType.kBrushless);
    CANSparkMax secondaryLiftMotor = new CANSparkMax(ElevatorConstants.rightElavatorMotor_ID, MotorType.kBrushless);

    RelativeEncoder liftEncoder;

    SparkMaxPIDController liftPidController;

    int smartMotionProfile = 1;

    double pid_setPoint = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxRPM, maxAcc, allowedErr;

    public ElevatorSubsystem() {
        liftMotor.restoreFactoryDefaults();
        secondaryLiftMotor.restoreFactoryDefaults();
        liftMotor.setInverted(false);
        secondaryLiftMotor.setInverted(true);
        liftMotor.setSmartCurrentLimit(40);
        secondaryLiftMotor.setSmartCurrentLimit(40);
        liftMotor.setClosedLoopRampRate(1);

        liftMotor.setIdleMode(IdleMode.kBrake);
        secondaryLiftMotor.setIdleMode(IdleMode.kBrake);

        liftEncoder = liftMotor.getEncoder();

        liftEncoder.setPosition(0);

        secondaryLiftMotor.follow(liftMotor, true);

        liftPidController = liftMotor.getPIDController();

        kP = ElevatorConstants.kP;
        kI = ElevatorConstants.kI;
        kD = ElevatorConstants.kD;
        kIz = ElevatorConstants.kIz;
        kFF = ElevatorConstants.kFF;
        kMaxOutput = ElevatorConstants.kMaxOutput;
        kMinOutput = ElevatorConstants.kMinOutput;
        maxRPM = ElevatorConstants.maxRPM;
        maxVel = ElevatorConstants.maxVel;
        minVel = ElevatorConstants.minVel;
        maxAcc = ElevatorConstants.maxAcc;
        allowedErr = ElevatorConstants.allowedErr;

        liftPidController.setP(kP, smartMotionProfile);
        liftPidController.setI(kI, smartMotionProfile);
        liftPidController.setD(kD, smartMotionProfile);
        liftPidController.setIZone(kIz, smartMotionProfile);
        liftPidController.setFF(kFF, smartMotionProfile);
        liftPidController.setOutputRange(kMinOutput, kMaxOutput, smartMotionProfile);
        liftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        liftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        liftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        liftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);
    }

    @Override
    public void periodic() {
        //
        SmartDashboard.putNumber("elevator position",liftEncoder.getPosition());
    }

    public void manualMoveElevator(double move) {
        liftMotor.set(move);
    }

    public void startElevatorSmartMotion() {
        liftPidController.setReference(pid_setPoint, ControlType.kSmartMotion, smartMotionProfile);
    }

    public void setLow() {
        pid_setPoint = 0;
    }

    public void setMid() {
        // 1.653543*PI/4 = inches per rotation
        pid_setPoint = 16;
    }

    public void setHigh() {
        pid_setPoint = 32;
    }

}
