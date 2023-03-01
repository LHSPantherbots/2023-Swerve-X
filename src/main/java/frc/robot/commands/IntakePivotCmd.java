package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakePivotCmd extends CommandBase {
    String position;
    IntakePivotSubsystem intakePivot;
    public IntakePivotCmd(String position, IntakePivotSubsystem intakePivot) {
        this.intakePivot = intakePivot;
        this.position = position;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        this.intakePivot.resetController();
        if (this.position == "stow") {
            this.intakePivot.setPositionStow();
        } else if (this.position == "intakeCone") {
            this.intakePivot.setPositionintakeCone();
        } else if (this.position == "intakeCube") {
            this.intakePivot.setPositionintakeCube();
        } else if (this.position == "scoreCone") {
            this.intakePivot.setPositionScoreCone();
        } else if (this.position == "scoreCone3") {
            this.intakePivot.setLevelt3ConeScore();
        } else if (this.position == "scoreCone2") {
            this.intakePivot.setLevelt2ConeScore();
        } else if (this.position == "scoreCube3") {
            this.intakePivot.setLevelt3CubeScore();
        } else if (this.position == "scoreCube2") {
            this.intakePivot.setLevelt2CubeScore();
        } else if (this.position == "intakeConeDoubleStation") {
            this.intakePivot.setPositonIntakeConeDoubleSubstation();
        }

    }

    @Override
    public void execute() {
        this.intakePivot.closedLoopIntakePivot();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePivot.closedLoopIntakePivot();
    }

    @Override
    public boolean isFinished() {
        return this.intakePivot.isAtPosition();
    }
}
