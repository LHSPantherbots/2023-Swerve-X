package frc.robot.commands;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Leds;

public class CoreStationDriveOnlyCharge extends SequentialCommandGroup {
    
    public CoreStationDriveOnlyCharge(
        DriveSubsystem drivesubsystem,
        Leds leds)
        {
        PathPlannerTrajectory path =
        PathPlanner.loadPath("CoreStationDriveOnlyCharge", new PathConstraints(3, 2), false);
        HashMap<String, Command> eventMap = new HashMap<>();
        
    SwerveAutoBuilder autoBuilder =
    new SwerveAutoBuilder(
        drivesubsystem::getPose,
        drivesubsystem::resetOdometry,
        Constants.DriveConstants.kDriveKinematics,
        new PIDConstants(
            5.0, 0.0,
            0.0), // PID constants to correct for translation error (used to create the X and Y
        // PID controllers)
        new PIDConstants(
            3.0, 0.0,
            0.0), // PID constants to correct for rotation error (used to create the rotation
        // controller),
        drivesubsystem::setModuleStates,
        eventMap,
        true,
        drivesubsystem);
        addCommands(
            new SequentialCommandGroup(
            new InstantCommand(() -> drivesubsystem.resetOdometry(path.getInitialPose())),
            autoBuilder.fullAuto(path),
            new AutoBalance2(drivesubsystem),
            new InstantCommand(drivesubsystem::restAll180, drivesubsystem)));

    }
}
