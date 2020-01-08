package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake.Direction;

import static frc.robot.Robot.intake;

public class ToggleIntake extends CommandBase {
    private Direction direction;

    public ToggleIntake(Direction direction) {
        addRequirements(intake);
        this.direction = direction;
    }

    public ToggleIntake() {
        addRequirements(intake);
        if (intake.getPosition() == Direction.UP) {
            direction = Direction.DOWN;
            return;
        }
        direction = Direction.UP;
    }

    @Override
    public void initialize() {
        intake.setPosition(direction);
    }

    @Override
    public void execute() {
        intake.changePositionAutomatically();
    }



    @Override
    public boolean isFinished() {
        return intake.getPosition() == direction;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
