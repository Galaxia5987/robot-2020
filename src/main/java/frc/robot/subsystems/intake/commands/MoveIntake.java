package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

/**
 * Close or open the intake mechanism.
 */
public class MoveIntake extends InstantCommand {
    private Intake intake;
    private State state;

    public MoveIntake(Intake intake, State state) {
        addRequirements(intake);
        this.intake = intake;
        this.state = state;
    }

    public MoveIntake(Intake intake) {
        addRequirements(intake);
        this.intake = intake;
        this.state = State.TOGGLE;
    }

    @Override
    public void initialize() {
        intake.setPosition(state);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
