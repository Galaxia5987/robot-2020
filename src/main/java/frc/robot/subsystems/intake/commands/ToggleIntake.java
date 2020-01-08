package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.intake;

public class ToggleIntake extends CommandBase {
    private Value direction;
    private boolean auto = false;

    public ToggleIntake(Value direction) {
        addRequirements(intake);
        this.direction = direction;
    }

    public ToggleIntake() {
        addRequirements(intake);
        this.auto = true;
    }

    private void chooseShootingType() {
        if (auto)
            intake.togglePosition();
        else
            intake.setPosition(direction);
    }

    @Override
    public void initialize() {
        chooseShootingType();
    }

    @Override
    public void execute() {
        chooseShootingType();
    }

    @Override
    public boolean isFinished() {
        return intake.getPosition() == direction;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
