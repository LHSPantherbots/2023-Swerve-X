package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Position;

public class ElevatorCmd extends CommandBase {
    Position position;
    ElevatorSubsystem elevator;

    public ElevatorCmd(Position position, ElevatorSubsystem elevator) {
        this.position = position;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.resetController();
        switch (this.position) {
            case STOW:
                this.elevator.setHeightStow();
                break;
            case CUBE_INTAKE:
                this.elevator.setHeightLow();
                break;
            case CONE_INTAKE:
                this.elevator.setHeightLow();
                break;
            case CONE_STATION_INTAKE:
                this.elevator.setHeightConeIntakeDoubleSubstation();
                break;
            case CUBE_SCORE_MID:
                this.elevator.setLevelt2CubeScore();
                break;
            case CUBE_SCORE_HIGH:
                this.elevator.setLevelt3CubeScore();
                break;
            case CONE_SCORE_MID:
                this.elevator.setLevelt2ConeScore();
                break;
            case CONE_SCORE_HIGH:
                this.elevator.setLevelt3ConeScore();
                break;
        }
    }

    @Override
    public void execute() {
        this.elevator.closedLoopElevator();

    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.closedLoopElevator();

    }

    @Override
    public boolean isFinished() {
        return this.elevator.isAtHeight();
    }

}
