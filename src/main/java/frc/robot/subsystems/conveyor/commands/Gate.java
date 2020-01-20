package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.subsystems.conveyor.Conveyor.gate;

public class Gate extends CommandBase {
    private boolean open;

    public Gate(Conveyor conveyor, boolean open){
        addRequirements(conveyor);
        this.open = open;
    }

    @Override
    public void initialize() {
        gate.set(open);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return open == gate.get();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
