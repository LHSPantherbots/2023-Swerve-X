package frc.robot.commands;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PPAuto extends CommandBase {
  // PathPlannerTrajectory path = PathPlanner.loadPath("LCube-LCubePick-MCube-Balance", new
  // PathConstraints(4, 3));

  // HashMap<String, Command> eventMap = new HashMap<>();

  // public PPAuto(SwerveAutoBuilder autoBuilder) {
  //     eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  //     autoBuilder.followPathWithEvents(path)
  // }
  Timer timer = new Timer();
  PathPlannerTrajectory trajectory;
  HolonomicDriveController controller;
  boolean resetOdometry;

  String pathName;

  public PPAuto(String pathname) {
    this(
        pathname,
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        true);
  }

  public PPAuto(String pathname, double maxVel, double maxAccel) {
    this(pathname, maxVel, maxAccel, true);
  }

  public PPAuto(String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
    addRequirements(RobotContainer.driveTrain);

    this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.controller = new HolonomicDriveController(xController, yController, thetaController);
    this.resetOdometry = resetOdometry;

    this.pathName = pathName;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Pose2d initialPose = trajectory.getInitialPose();
    if (resetOdometry) {
      new Rotation2d();
      RobotContainer.driveTrain.resetOdometry(
          new Pose2d(
              initialPose.getTranslation(),
              Rotation2d.fromDegrees(RobotContainer.driveTrain.getHeading())));
    }
  }

  @Override
  public void execute() {
    double time = timer.get();
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
    ChassisSpeeds targetSpeeds =
        controller.calculate(
            RobotContainer.driveTrain.getPose(),
            desiredState,
            new Rotation2d(desiredState.holonomicRotation.getRadians()));

    targetSpeeds.vyMetersPerSecond = -targetSpeeds.vyMetersPerSecond;
    targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;

    Pose2d currentPose = RobotContainer.driveTrain.getPose();
    String tString = " [" + Math.round(timer.get() * 100) / 100.0 + "]";
    System.out.println(
        pathName + tString + " x error: " + (desiredState.poseMeters.getX() - currentPose.getX()));
    System.out.println(
        pathName + tString + " y error: " + (desiredState.poseMeters.getY() - currentPose.getY()));
    System.out.println(
        pathName
            + tString
            + " r error: "
            + (desiredState.holonomicRotation.getDegrees()
                - currentPose.getRotation().getDegrees()));

    RobotContainer.driveTrain.drive(
        targetSpeeds.vxMetersPerSecond,
        targetSpeeds.vyMetersPerSecond,
        targetSpeeds.omegaRadiansPerSecond,
        true);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.driveTrain.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
