package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.Constants.Conveyor.CONVEYOR_RETURN_POWER;

/**
 * Move all of the Power Cells from the mechanical stopper to the intake proximity, to prevent gaps between them
 */
public class MinimizeConveyor extends CommandBase {

    private final Conveyor conveyor;

    public MinimizeConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setPower(CONVEYOR_RETURN_POWER);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return conveyor.intakeSensedBall();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}