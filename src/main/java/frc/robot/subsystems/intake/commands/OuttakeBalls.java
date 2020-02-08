package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

public class OuttakeBalls extends CommandBase {
    private final double conveyorSpeed;
    private Intake intake;
    private Conveyor conveyor;
    private double speed;

    /**
     * this constructor is for a situation when you use an autonomous command.
     * please be aware that you should insert positive {@param speed)'s number.
     *
     * @param speed
     */
    public OuttakeBalls(Conveyor conveyor, Intake intake, double speed, double conveyorSpeed) {
        addRequirements(intake, conveyor);
        this.conveyor = conveyor;
        this.intake = intake;
        this.speed = -speed;
        this.conveyorSpeed = -conveyorSpeed;
    }

    @Override
    public void initialize() {
        intake.setPosition(State.OPEN);
        intake.powerWheels(speed);
        conveyor.setFunnelPower(speed);
        conveyor.setConveyorPower(conveyorSpeed);
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
        conveyor.setPower(0);
    }
}
