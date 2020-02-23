package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;

import static frc.robot.Constants.Conveyor.*;

/**
 * Turn the conveyor and close the gate
 */
public class LoadConveyor extends CommandBase {
    private Conveyor conveyor;

    public LoadConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setGate(State.CLOSE);
        conveyor.setConveyorPower(CONVEYOR_INTAKE_POWER);
        conveyor.setFunnelPower(FUNNEL_INTAKE_POWER);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}