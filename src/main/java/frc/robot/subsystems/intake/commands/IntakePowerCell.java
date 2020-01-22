package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.Constants.Conveyor.MAX_BALLS_AMOUNT;
import static frc.robot.RobotContainer.intake;

public class IntakePowerCell extends CommandBase {
    private double speed;
    private Intake intake;
    private Conveyor conveyor;

    public IntakePowerCell(Intake intake, Conveyor conveyor, double speed) {
        addRequirements(intake, conveyor);
        this.intake = intake;
        this.conveyor = conveyor;
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
        return conveyor.getBallsCount() >= MAX_BALLS_AMOUNT;
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
