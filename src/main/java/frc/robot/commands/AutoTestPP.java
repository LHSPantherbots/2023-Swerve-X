package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;

public class AutoTestPP extends SequentialCommandGroup {
    public AutoTestPP(CrossSlideSubsystem crossSlide, IntakePivotSubsystem intakePivot, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
            new ConeScoreHigh(crossSlide, intakePivot, elevatorSubsystem),
            new ParallelDeadlineGroup(
                new PPAuto("LCube-LCubePick-MCube-Balance", 3, 4), 
                new CubeIntakeGround(crossSlide, intakePivot, elevatorSubsystem))
        );
    }
}
