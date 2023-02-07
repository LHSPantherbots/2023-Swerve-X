package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TwoCubeAuto extends SequentialCommandGroup {
    private final DriveSubsystem m_DriveSubsystem;
    private final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    private final Trajectory tj1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.075, 0)
        ),
        new Pose2d(0.152, 0, new Rotation2d(0)),
        config);
    private final Trajectory tj2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.152, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-0.9, -0.152)
        ),
        new Pose2d(-4.45, -0.152, new Rotation2d(1)),
        config);
    private final Trajectory tj3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-4.45, -0.152, new Rotation2d(1)),
        List.of(
            new Translation2d(-0.9, -0.152)
        ),
        new Pose2d(0, 1.676, new Rotation2d(0)),
        config);
    private final Trajectory tj4 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 1.676, new Rotation2d(0)),
        List.of(
            new Translation2d(0.075, 1.676)
        ),
        new Pose2d(0.152, 1.676, new Rotation2d(0)),
        config);
    private final Trajectory tj5 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.152, 1.676, new Rotation2d(0)),
        List.of(
            new Translation2d(0.919, 1.676)
        ),
        new Pose2d(-1.838, 1.676, new Rotation2d(0)),
        config);

    private ProfiledPIDController thetaController;
    
    public TwoCubeAuto(DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(subsystem);

        thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand scc1 =
            new SwerveControllerCommand(
                tj1,
                m_DriveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_DriveSubsystem::setModuleStates,
                m_DriveSubsystem);
        
        SwerveControllerCommand scc2 =
            new SwerveControllerCommand(
                tj2,
                m_DriveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_DriveSubsystem::setModuleStates,
                m_DriveSubsystem);
        
        SwerveControllerCommand scc3 =
            new SwerveControllerCommand(
                tj3,
                m_DriveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_DriveSubsystem::setModuleStates,
                m_DriveSubsystem);

        SwerveControllerCommand scc4 =
            new SwerveControllerCommand(
                tj4,
                m_DriveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_DriveSubsystem::setModuleStates,
                m_DriveSubsystem);
        SwerveControllerCommand scc5 =
            new SwerveControllerCommand(
                tj5,
                m_DriveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_DriveSubsystem::setModuleStates,
                m_DriveSubsystem);
        addCommands(
            new InstantCommand(m_DriveSubsystem::resetEncoders, m_DriveSubsystem),
            new InstantCommand(m_DriveSubsystem::zeroHeading, m_DriveSubsystem),
            new InstantCommand(() -> m_DriveSubsystem.resetOdometry(tj1.getInitialPose()), m_DriveSubsystem),
            scc1.andThen(() -> m_DriveSubsystem.drive(0, 0, 0, false)),
            // command(s) to score cube here
            scc2.andThen(() -> m_DriveSubsystem.drive(0, 0, 0, false)),
            // command(s) to pickup cube here
            scc3.andThen(() -> m_DriveSubsystem.drive(0, 0, 0, false)),
            scc4.andThen(() -> m_DriveSubsystem.drive(0, 0, 0, false)),
            // command(s) to score cube here
            scc5.andThen(() -> m_DriveSubsystem.drive(0, 0, 0, false))
            //command to balance on powerstation here
        );
        
    }
}
