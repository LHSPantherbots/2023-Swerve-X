package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCmd extends CommandBase {
    String position;
    ElevatorSubsystem elevator;
    public ElevatorCmd(String position, ElevatorSubsystem elevator) {
        this.position = position;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.resetController();
        if (this.position == "High") {
            this.elevator.setHeightHigh();
        } else if (this.position == "Mid") {
            this.elevator.setHeightMid();
        } else if (this.position == "Low") {
            this.elevator.setHeightLow();
        } else if (this.position == "stow") {
            this.elevator.setHeightStow();
        } else if (this.position == "Cone3") {
            this.elevator.setLevelt3ConeScore();
        } else if (this.position == "Cone2") {
            this.elevator.setLevelt2ConeScore();
        } else if (this.position == "Cube3") {
            this.elevator.setLevelt3CubeScore();
        } else if (this.position == "Cube2") {
            this.elevator.setLevelt2CubeScore();
        } else if (this.position == "DoubleSubstation") {
            this.elevator.setHeightConeIntakeDoubleSubstation();
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
