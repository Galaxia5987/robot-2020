package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;


public class MoveGate extends InstantCommand {
    private boolean open;
    private Conveyor conveyor;

    public MoveGate(Conveyor conveyor, boolean open){
        addRequirements(conveyor);
        this.open = open;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        conveyor.setGate(open);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }
}
