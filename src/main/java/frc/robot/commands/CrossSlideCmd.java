package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CrossSlideSubsystem;

public class CrossSlideCmd extends CommandBase{
    CrossSlideSubsystem crosSlide;
    String position;
    public CrossSlideCmd(String position, CrossSlideSubsystem crossSlide) {
        this.crosSlide = crossSlide;
        this.position = position;
        addRequirements(crossSlide);
    }

    @Override
    public void initialize() {
        this.crosSlide.resetController();
        if (this.position == "stow") {
            this.crosSlide.setPositionStow();
        } else if (this.position == "intake") {
            this.crosSlide.setPositionIntake();
        } else if (this.position == "out") {
            this.crosSlide.setPositionOut();
        } else if (this.position == "Cone3") {
            this.crosSlide.setLevelt3ConeScore();
        } else if (this.position == "Cone2") {
            this.crosSlide.setLevelt2ConeScore();
        } else if (this.position == "Cube3") {
            this.crosSlide.setLevelt3CubeScore();
        } else if (this.position == "Cube2") {
            this.crosSlide.setLevelt2CubeScore();
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
