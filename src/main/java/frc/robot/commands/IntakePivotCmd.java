package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.util.Position;

public class IntakePivotCmd extends CommandBase {
    Position position;
    IntakePivotSubsystem intakePivot;
    boolean finishes;

    public IntakePivotCmd(Position position, IntakePivotSubsystem intakePivot) {
        this.intakePivot = intakePivot;
        this.position = position;
        this.finishes = true;
        addRequirements(intakePivot);
    }

    public IntakePivotCmd(Position position, IntakePivotSubsystem intakePivot, boolean finishes) {
        this.intakePivot = intakePivot;
        this.position = position;
        this.finishes = finishes;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        this.intakePivot.resetController();
        switch (this.position) {
            case STOW:
                this.intakePivot.setPositionStow();
                break;
            case CUBE_INTAKE:
                this.intakePivot.setPositionintakeCube();
                break;
            case CONE_INTAKE:
                this.intakePivot.setPositionintakeCone();
                break;
            case CONE_STATION_INTAKE:
                this.intakePivot.setPositonIntakeConeDoubleSubstation();
                break;
            case CUBE_SCORE_MID:
                this.intakePivot.setLevelt2CubeScore();
                break;
            case CUBE_SCORE_HIGH:
                this.intakePivot.setLevelt3CubeScore();
                break;
            case CONE_SCORE_MID:
                this.intakePivot.setLevelt2ConeScore();
                break;
            case CONE_SCORE_HIGH:
                this.intakePivot.setLevelt3ConeScore();
                break;
            case HOLD:
                this.intakePivot.closedLoopIntakePivot();
                break;
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
        if(finishes){
            return this.intakePivot.isAtPosition();
        }else{
            return false;
        }
    }
}
