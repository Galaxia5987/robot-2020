package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

import static frc.robot.Constants.Intake.INTAKE_POWER;

public class IntakeBalls extends CommandBase {
    private Intake intake;
    private Conveyor conveyor = null;

    public IntakeBalls(Intake intake) {
        addRequirements(intake);
        this.intake = intake;
    }

    /**
     * Second constructor, where conveyor is given, and the isFinished works based on the ball count.
     *
     * @param intake   intake subsystem
     * @param conveyor conveyor subsystem (optional)
     */
    public IntakeBalls(Intake intake, Conveyor conveyor) {
        addRequirements(intake, conveyor);
        this.intake = intake;
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        intake.setPosition(State.OPEN);
        intake.powerWheels(INTAKE_POWER.get());
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (conveyor != null) {
            final boolean buttonNotPressed = !OI.a.get();
            return conveyor.getBallsCount() >= 5 && buttonNotPressed;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
