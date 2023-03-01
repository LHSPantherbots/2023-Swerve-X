package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrossSlideSubsystem;
import frc.robot.util.Position;

public class CrossSlideCmd extends CommandBase {
    CrossSlideSubsystem crosSlide;
    Position position;

    public CrossSlideCmd(Position position, CrossSlideSubsystem crossSlide) {
        this.crosSlide = crossSlide;
        this.position = position;
        addRequirements(crossSlide);
    }

    @Override
    public void initialize() {
        this.crosSlide.resetController();
        switch (this.position) {
            case STOW:
                this.crosSlide.setPositionStow();
                ;
                break;
            case CUBE_INTAKE:
                this.crosSlide.setPositionIntake();
                break;
            case CONE_INTAKE:
                this.crosSlide.setPositionIntake();
                break;
            case CONE_STATION_INTAKE:
                this.crosSlide.setPositionIntake();
                break;
            case CUBE_SCORE_MID:
                this.crosSlide.setLevelt2CubeScore();
                break;
            case CUBE_SCORE_HIGH:
                this.crosSlide.setLevelt3CubeScore();
                break;
            case CONE_SCORE_MID:
                this.crosSlide.setLevelt2ConeScore();
                break;
            case CONE_SCORE_HIGH:
                this.crosSlide.setLevelt3ConeScore();
                break;
        }

    }

    @Override
    public void execute() {
        // this.crosSlide.setPositionStow();
        this.crosSlide.closedLoopCrossSlide();
    }

    @Override
    public void end(boolean interrupted) {
        this.crosSlide.closedLoopCrossSlide();
    }

    @Override
    public boolean isFinished() {
        return this.crosSlide.isAtPosition();
    }
}
