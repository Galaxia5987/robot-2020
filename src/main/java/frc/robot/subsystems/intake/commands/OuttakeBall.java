package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

public class OuttakeBall extends CommandBase {
    private Intake intake;
    private Conveyor conveyor;
    private double speed;

    /**
     * this constructor is for a situation when you use an autonomous command.
     * please be aware that you should insert positive {@param speed)'s number.
     *
     * @param speed
     */
    public OuttakeBall(Conveyor conveyor, Intake intake, double speed) {
        addRequirements(intake, conveyor);
        this.conveyor = conveyor;
        this.intake = intake;
        this.speed = -speed;
    }

    @Override
    public void initialize() {
        intake.setPosition(false);
        intake.powerWheels(speed);
        conveyor.setConveyorSpeed(speed);
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
        conveyor.setConveyorSpeed(0);
    }
}
