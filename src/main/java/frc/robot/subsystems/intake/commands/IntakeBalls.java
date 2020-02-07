package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

public class IntakeBalls extends CommandBase {
    private double power;
    private Intake intake;
    private Conveyor conveyor = null;

    public IntakeBalls(Intake intake, double power) {
        addRequirements(intake);
        this.intake = intake;
        this.power = power;
    }

    /**
     * Second constructor, where conveyor is given, and the isFinished works based on the ball count.
     * @param intake intake subsystem
     * @param conveyor conveyor subsystem (optional)
     * @param power (motor power)
     */
    public IntakeBalls(Intake intake, Conveyor conveyor, double power){
        this(intake, power);
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        intake.setPosition(false);
        intake.powerWheels(power);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if(conveyor != null) {
            final boolean buttonNotPressed = !OI.a.get();
            return conveyor.getBallsCount() >= 5 && buttonNotPressed;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
