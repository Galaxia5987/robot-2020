package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.intake;

public class ToggleIntake extends CommandBase {
    private boolean direction;
    private boolean auto = false;

    public ToggleIntake(boolean up) {
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
        chooseFoldingState();
    }

    @Override
    public boolean isFinished() {
        if (direction)
            return intake.isFolded();
        return !intake.isFolded();
    }

    @Override
    public void end(boolean interrupted) {

    }
}