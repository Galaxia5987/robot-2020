package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.RobotContainer.intake;

public class IntakePowerCell extends CommandBase {
    private double speed;

    public IntakePowerCell(Intake intake, double speed) {
        addRequirements(intake);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setPosition(false);
        intake.powerWheels(speed);
    }

    @Override
    public void execute() {
        intake.powerWheels(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
