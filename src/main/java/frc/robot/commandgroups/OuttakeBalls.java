package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

public class OuttakeBalls extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;

    /**
     * this constructor is for a situation when you use an autonomous command.
     * please be aware that you should insert positive {@param speed)'s number.
     *
     * @param speed
     */
    public OuttakeBalls(Conveyor conveyor, Intake intake) {
        addRequirements(intake, conveyor);
        this.conveyor = conveyor;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setPosition(State.CLOSE);
        intake.powerWheels(-Constants.Intake.OUTTAKE_POWER.get());
        conveyor.setFunnelPower(-Constants.Conveyor.FUNNEL_OUTTAKE_POWER.get());
        conveyor.setConveyorPower(-Constants.Conveyor.CONVEYOR_OUTTAKE_POWER.get());
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
        intake.powerWheels(0);
        conveyor.stopAll();
    }
}
