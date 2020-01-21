package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.intake;

public class OuttakeBall extends CommandBase {
    private double speed;

    /**
     * this constructor is for a situation when you use an autonomous command.
     * please be aware that you should insert positive {@param speed)'s number.
     *
     * @param speed
     */
    public OuttakeBall(double speed) {
        addRequirements(intake);
        this.speed = -speed;
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
