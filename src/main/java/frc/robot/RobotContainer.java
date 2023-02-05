// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePadButtons;
import frc.robot.Constants.OIConstants;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Talon and Pigeon needed for subsystems defined here...
  // The robot's subsystems and commands are defined here...
  public final static LimeLight limelight = new LimeLight();
  public final static DriveSubsystem driveTrain = new DriveSubsystem();
  public final static Leds leds = new Leds();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


 // The driver's controller
 XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

          leds.setDefaultCommand(
              new RunCommand(() -> leds.rainbow(), leds)
          );

        // Set the default drive command to split-stick arcade drive
        driveTrain.setDefaultCommand(
          // A split-stick arcade command, with forward/backward controlled by the left
          // hand, and turning controlled by the right.
          new RunCommand(
              () ->
                  driveTrain.drive(
                    -m_driverController.getLeftY()
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                      -m_driverController.getLeftX()
              * DriveConstants.kMaxSpeedMetersPerSecond,
                     // -m_driverController.getRightX()
                     -(m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis())
              * DriveConstants.kMaxSpeedMetersPerSecond,
                      
                      true), driveTrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, GamePadButtons.Start)
    .whileTrue(new InstantCommand(driveTrain::resetAll, driveTrain));
    

    new JoystickButton(m_driverController, GamePadButtons.Y)
    .onTrue(new InstantCommand(limelight::ledPipeline, limelight))
    .onTrue(new InstantCommand(limelight::setPipelineThree, limelight))
    .whileTrue(new RunCommand(() -> driveTrain.limeLightAim(
                    -m_driverController.getLeftY()
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                      -m_driverController.getLeftX()
              * DriveConstants.kMaxSpeedMetersPerSecond), driveTrain))
    .onFalse(new InstantCommand(limelight::setPipelineZero, limelight));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(.5, .5), new Translation2d(1.0, -.5)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.5, 0, new Rotation2d(Math.PI)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            driveTrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false));
  }
}
