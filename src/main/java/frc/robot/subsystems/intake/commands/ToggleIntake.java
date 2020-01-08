package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.intake;

public class ToggleIntake extends CommandBase {
    private Value direction;

    public ToggleIntake(Value direction) {
        addRequirements(intake);
        this.direction = direction;
    }

    public ToggleIntake() {
        addRequirements(intake);
        if (intake.getPosition() == Value.kForward) {
            direction = Value.kReverse;
            return;
        }
        direction = Value.kForward;
    }

    @Override
    public void initialize() {
        intake.setPosition(direction);
    }

    @Override
    public void execute() {
        intake.togglePosition();
    }

    @Override
    public boolean isFinished() {
        return intake.getPosition() == direction;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
