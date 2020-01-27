package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

import static frc.robot.Constants.Conveyor.MAX_BALLS_AMOUNT;

public class IntakeBalls extends CommandBase {
    private double speed;
    private Intake intake;

    public IntakeBalls(Intake intake, double speed) {
        addRequirements(intake);
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setPosition(false);
        intake.powerWheels(speed);
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
    }
}
