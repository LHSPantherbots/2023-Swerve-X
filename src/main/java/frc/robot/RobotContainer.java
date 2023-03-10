// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePadButtons;
import frc.robot.Constants.OIConstants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoConeHigh;
import frc.robot.commands.AutoCubeHigh;
import frc.robot.commands.AutoHighScoreConeBalance;
import frc.robot.commands.AutoHighScoreConeCubeIntake;
import frc.robot.commands.AutoHighScoreCubeBalance;
import frc.robot.commands.ConeIntakeDoubleSubstation;
import frc.robot.commands.ConeIntakeGround;
import frc.robot.commands.ConeScoreHigh;
import frc.robot.commands.ConeScoreMid;
import frc.robot.commands.CrossSlideCmd;
import frc.robot.commands.CubeIntakeGround;
import frc.robot.commands.CubeScoreHigh;
import frc.robot.commands.CubeScoreMid;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.HoldAtCurrentPosition;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.IntakePivotCmd;
import frc.robot.commands.PPEventTest;
import frc.robot.commands.StowAll;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.util.Position;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  public static final RobotStateSubsystem robotState = new RobotStateSubsystem();
  public static final LimeLight limelight = new LimeLight();
  public static final DriveSubsystem driveTrain = new DriveSubsystem();
  public static final Leds leds = new Leds();
  public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static final CrossSlideSubsystem crossSlide = new CrossSlideSubsystem();
  public static final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public static SendableChooser<Command> autoChoice = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add Button Controls to Smart Dashboard
    SmartDashboard.putData(
        "Elevator Mid",
        new InstantCommand(elevator::resetController, elevator)
            .andThen(new RunCommand(elevator::setHeightMid, elevator)));
    SmartDashboard.putData(
        "Elevator Low",
        new InstantCommand(elevator::resetController, elevator)
            .andThen(new RunCommand(elevator::setHeightLow, elevator)));
    SmartDashboard.putData(
        "Elevator Stop",
        new InstantCommand(elevator::resetController, elevator)
            .andThen(new RunCommand(elevator::stopElevator, elevator)));

    SmartDashboard.putData(
        "Cross Slide In",
        new InstantCommand(crossSlide::resetController, crossSlide)
            .andThen(new RunCommand(crossSlide::setPositionIntake, crossSlide)));
    SmartDashboard.putData(
        "Cross Slide Out",
        new InstantCommand(crossSlide::resetController, crossSlide)
            .andThen(new RunCommand(crossSlide::setPositionOut, crossSlide)));

    SmartDashboard.putData(
        "Pivot Up",
        new InstantCommand(intakePivot::resetController, intakePivot)
            .andThen(new RunCommand(intakePivot::setPositionStow, intakePivot)));
    SmartDashboard.putData(
        "Pivot Down",
        new InstantCommand(intakePivot::resetController, intakePivot)
            .andThen(new RunCommand(intakePivot::setPositionScoreCone, intakePivot)));

    SmartDashboard.putData(
        "Cone Intake Command", new ConeIntakeGround(crossSlide, intakePivot, elevator));
    SmartDashboard.putData("Stow All", new StowAll(crossSlide, intakePivot, elevator));

    SmartDashboard.putData("LEDS Off", new RunCommand(leds::ledsOff, leds));
    SmartDashboard.putData("Panther Streak LED", new RunCommand(leds::pantherStreak, leds));
    SmartDashboard.putData("Blue LED", new RunCommand(leds::blue, leds));
    SmartDashboard.putData("Red LED", new RunCommand(leds::red, leds));
    SmartDashboard.putData("Green LED", new RunCommand(leds::green, leds));

    SmartDashboard.putData("Cone Score: Mid ", new ConeScoreMid(crossSlide, intakePivot, elevator));
    SmartDashboard.putData(
        "Cone Score: High ", new ConeScoreHigh(crossSlide, intakePivot, elevator));

    SmartDashboard.putData("Eject Cone", new RunCommand(intake::ejectCone, intake));

    SmartDashboard.putData(
        "Auto Cone High", new AutoConeHigh(elevator, crossSlide, intakePivot, intake));

    SmartDashboard.putData(
        "Hold Position", new HoldAtCurrentPosition(crossSlide, intakePivot, elevator));
    SmartDashboard.putData(
        "Auto Cube High", new AutoCubeHigh(elevator, crossSlide, intakePivot, intake));

    SmartDashboard.putData("Intake Hold", new IntakeHold(intake));

    SmartDashboard.putData("Auto Balance", new AutoBalance(driveTrain, elevator));

    Shuffleboard.getTab("Autonomous").add(autoChoice);
    autoChoice.addOption("Do Nothing", new RunCommand(() -> driveTrain.drive(0, 0, 0, true)));
    autoChoice.addOption(
        "High Cone Balance",
        new AutoHighScoreConeBalance(driveTrain, elevator, crossSlide, intakePivot, intake));
    autoChoice.addOption(
        "High Cube Balance",
        new AutoHighScoreCubeBalance(driveTrain, elevator, crossSlide, intakePivot, intake));
    autoChoice.addOption(
        "High Cone Drive to Center",
        new AutoHighScoreConeCubeIntake(driveTrain, elevator, crossSlide, intakePivot, intake));
    autoChoice.addOption(
        "Auto Cone Hight", new AutoConeHigh(elevator, crossSlide, intakePivot, intake));
    autoChoice.addOption(
        "EventTest", new PPEventTest(elevator, crossSlide, intakePivot, intake, driveTrain, leds));
    //    new AutoDriveAndRot(elevator, crossSlide, intakePivot, intake, driveTrain));
    // autoChoice.addOption(
    //    "AutoConeCubeTopBalance",
    //    new AutoConeCubeTopBalance(elevator, crossSlide, intakePivot, intake, driveTrain));
    // autoChoice.addOption(
    //    "AutoConeCubeBottomBalance",
    //    new AutoConeCubeBottomBalance(elevator, crossSlide, intakePivot, intake, driveTrain));

    // Configure the button bindings
    configureButtonBindings();

    leds.setDefaultCommand(new RunCommand(() -> leds.pantherStreak(), leds));

    elevator.setDefaultCommand(new ElevatorCmd(Position.HOLD, elevator, false));

    crossSlide.setDefaultCommand(new CrossSlideCmd(Position.HOLD, crossSlide, false));

    intakePivot.setDefaultCommand(new IntakePivotCmd(Position.HOLD, intakePivot, false));

    intake.setDefaultCommand(
        // new RunCommand(intake::stopIntake, intake)
        new IntakeHold(intake));

    // Set the default drive command to split-stick arcade drive
    driveTrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                driveTrain.drive(
                    -m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond,
                    // -m_driverController.getRightX()
                    -(m_driverController.getRightTriggerAxis()
                            - m_driverController.getLeftTriggerAxis())
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                    true),
            driveTrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER CONTROLS

    new JoystickButton(m_driverController, GamePadButtons.Start)
        .whileTrue(new InstantCommand(driveTrain::resetAll, driveTrain));

    // While the left bumper is held down, the robot's speed will be set to a tenth of its standard
    // value,
    // and the leds will pulse orange to indicate reduced speed
    double reducedspeedconstant = 0.2; // how much the speed is reduced by
    new JoystickButton(m_driverController, GamePadButtons.LB)
        .whileTrue(
            new RunCommand(
                () ->
                    driveTrain.drive(
                        -m_driverController.getLeftY()
                            * reducedspeedconstant
                            * DriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getLeftX()
                            * reducedspeedconstant
                            * DriveConstants.kMaxSpeedMetersPerSecond,
                        // -m_driverController.getRightX()
                        -(m_driverController.getRightTriggerAxis()
                                - m_driverController.getLeftTriggerAxis())
                            * reducedspeedconstant
                            * 2.0
                            * DriveConstants
                                .kMaxSpeedMetersPerSecond, // Doubled the rotation because it was
                        // not turning at reduced speed
                        true),
                driveTrain))
        .whileTrue(new RunCommand(leds::orangePulse, leds));

    new JoystickButton(
            m_driverController,
            GamePadButtons
                .A) // These dont move robot, but should change the limeilight state currently
        .onTrue(new InstantCommand(limelight::setPipelineOne, limelight))
        .onFalse(new InstantCommand(limelight::setPipelineZero, limelight));

    new JoystickButton(m_driverController, GamePadButtons.B)
        .onTrue(new InstantCommand(limelight::setPipelineTwo, limelight))
        .onFalse(new InstantCommand(limelight::setPipelineZero, limelight));

    // Operator Controls

    // Turns on Cone Mode
    new JoystickButton(operatorController, GamePadButtons.A)
        // .onTrue(new InstantCommand(() -> robotState.setConeMode(true), robotState))
        // .onTrue(new InstantCommand(() -> robotState.setCubeMode(false), robotState))
        .onTrue(new RunCommand(() -> robotState.setConeMode(true), robotState))
        .onTrue(new RunCommand(leds::yellow, leds));

    // Turns on Cube Mode
    new JoystickButton(operatorController, GamePadButtons.B)
        // .onTrue(new InstantCommand(() -> robotState.setConeMode(false), robotState))
        // .onTrue(new InstantCommand(() -> robotState.setCubeMode(true), robotState))
        .onTrue(new RunCommand(() -> robotState.setConeMode(false), robotState))
        .onTrue(new RunCommand(leds::purple, leds));

    // Runs Intake both Driver and operator have these buttons currently if there is a change do to
    // both
    new JoystickButton(operatorController, GamePadButtons.X)
        .whileTrue(
            new ConditionalCommand(
                new RunCommand(intake::intakeCone, intake), // runs if cone mode
                new RunCommand(
                    intake::intakeCube, intake), // runs if cube mode (or cone mode false)
                () -> robotState.getConeMode()));

    new JoystickButton(operatorController, GamePadButtons.Y)
        .whileTrue(
            new ConditionalCommand(
                new RunCommand(intake::ejectCone, intake), // runs if cone mode
                new RunCommand(intake::ejectCube, intake), // runs if cube mode (or cone mode false)
                () -> robotState.getConeMode()));

    new POVButton(operatorController, GamePadButtons.Left)
        .onTrue(new StowAll(crossSlide, intakePivot, elevator));

    new POVButton(operatorController, GamePadButtons.Down)
        .onTrue(
            new ConditionalCommand(
                new ConeIntakeGround(crossSlide, intakePivot, elevator), // runs if cone mode
                new CubeIntakeGround(
                    crossSlide, intakePivot, elevator), // runs if cube mode (or cone mode false)
                () -> robotState.getConeMode()));

    new POVButton(operatorController, GamePadButtons.Right)
        .onTrue(
            new ConditionalCommand(
                new ConeScoreMid(crossSlide, intakePivot, elevator), // runs if cone mode
                new CubeScoreMid(
                    crossSlide, intakePivot, elevator), // runs if cube mode (or cone mode false)
                () -> robotState.getConeMode()));

    new POVButton(operatorController, GamePadButtons.Up)
        .onTrue(
            new ConditionalCommand(
                new ConeScoreHigh(crossSlide, intakePivot, elevator), // runs if cone mode
                new CubeScoreHigh(
                    crossSlide, intakePivot, elevator), // runs if cube mode (or cone mode false)
                () -> robotState.getConeMode()));

    new JoystickButton(operatorController, GamePadButtons.Select)
        .onTrue(new ConeIntakeDoubleSubstation(crossSlide, intakePivot, elevator));

    new JoystickButton(operatorController, GamePadButtons.LB)
        .whileTrue(
            new RunCommand(
                () ->
                    elevator.manualElevator(
                        (operatorController.getRightTriggerAxis()
                                - operatorController.getLeftTriggerAxis())
                            * .1),
                elevator))
        .whileTrue(
            new RunCommand(
                () -> crossSlide.manualCrossSlide(operatorController.getLeftY() * .1), crossSlide))
        .whileTrue(
            new RunCommand(
                () -> intakePivot.manualintakePivot(-operatorController.getRightY() * .2),
                intakePivot))
        .onFalse(new HoldAtCurrentPosition(crossSlide, intakePivot, elevator));
  }

  public Command getInitCommand() {
    return new InstantCommand(intake::resetEncoder, intake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    // TrajectoryConfig rev_config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // .setReversed(true).setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(Math.PI)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(0.5,0)),
    //         //List.of(new Translation2d(.5, .5), new Translation2d(1.0, -.5)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(1.0, 0.0, new Rotation2d(Math.PI)),
    //         //config);
    //         rev_config);

    // var thetaController =
    //     new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    //     new SwerveControllerCommand(
    //         exampleTrajectory,
    //         driveTrain::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         driveTrain::setModuleStates,
    //         driveTrain);

    // // Reset odometry to the starting pose of the trajectory.
    // driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return autoChoice.getSelected(); // new InstantCommand(driveTrain::restAll180, driveTrain)
    // .andThen(
    //    new InstantCommand(() -> driveTrain.resetOdometry(driveTrain.getPose()), driveTrain)
    //  .andThen(swerveControllerCommand)
    //  .andThen(new InstantCommand(driveTrain::restAll180, driveTrain))
    //  .andThen(() -> driveTrain.drive(0, 0, 0, false));
  }
}
