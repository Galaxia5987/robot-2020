package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.RobotContainer.intake;

public class ToggleIntake extends InstantCommand {
    private Intake intake;
    private boolean direction;
    private boolean auto = false;

    public ToggleIntake(Intake intake, boolean up) {
        addRequirements(intake);
        this.direction = up;
    }

    public ToggleIntake() {
        addRequirements(intake);
        this.auto = true;
    }

    private void chooseFoldingState() {
        if (auto)
            intake.togglePosition();
        else
            intake.setPosition(direction);
    }

    @Override
    public void initialize() {
        chooseFoldingState();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }
}
