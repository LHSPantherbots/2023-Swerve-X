package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    CANSparkMax armMotor=new CANSparkMax(0, MotorType.kBrushless);
    RelativeEncoder armEncoder;

    SparkMaxPIDController armPidController;

    int smartMotionProfile = 1;

    double pid_setPoint = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxRPM, maxAcc, allowedErr;

    public ArmSubsystem() {
        armMotor.restoreFactoryDefaults();
        armMotor.setInverted(false);
        armMotor.setSmartCurrentLimit(40);
        armMotor.setClosedLoopRampRate(1);

        armMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = armMotor.getEncoder();

        armEncoder.setPosition(0);

        armPidController = armMotor.getPIDController();

        kP = ArmConstants.kP;
        kI = ArmConstants.kI;
        kD = ArmConstants.kD;
        kIz = ArmConstants.kIz;
        kFF = ArmConstants.kFF;
        kMaxOutput = ArmConstants.kMaxOutput;
        kMinOutput = ArmConstants.kMinOutput;
        maxRPM = ArmConstants.maxRPM;
        maxVel = ArmConstants.maxVel;
        minVel = ArmConstants.minVel;
        maxAcc = ArmConstants.maxAcc;
        allowedErr = ArmConstants.allowedErr;

        armPidController.setP(kP, smartMotionProfile);
        armPidController.setI(kI, smartMotionProfile);
        armPidController.setD(kD, smartMotionProfile);
        armPidController.setIZone(kIz, smartMotionProfile);
        armPidController.setFF(kFF, smartMotionProfile);
        armPidController.setOutputRange(kMinOutput, kMaxOutput, smartMotionProfile);
        armPidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        armPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        armPidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        armPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm position", armEncoder.getPosition());
    }
}
