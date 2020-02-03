package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

/**
 * Close or open the intake mechanism.
 */
public class MoveIntake extends InstantCommand {
    private Intake intake;
    private boolean direction;
    private boolean auto = false;

    public MoveIntake(Intake intake, boolean up) {
        addRequirements(intake);
        this.intake = intake;
        this.direction = up;
    }

    public MoveIntake(Intake intake) {
        addRequirements(intake);
        this.intake = intake;
        this.auto = true;
    }

    @Override
    public void initialize() {
        if (auto)
            intake.togglePosition();
        else
            intake.setPosition(direction);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }
}
