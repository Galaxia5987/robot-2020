package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.subsystems.conveyor.Conveyor.gate;

public class Gate extends CommandBase {
    private boolean direction;

    public Gate(Conveyor conveyor, boolean direction){
        addRequirements(conveyor);
        this.direction = direction;
    }

    @Override
    public void initialize() {
        gate.set(direction);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return direction == gate.get();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
