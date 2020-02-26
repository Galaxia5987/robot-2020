package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.State;

import static frc.robot.Constants.Intake.INTAKE_CONSTANT_VALUE;
import static frc.robot.Constants.Intake.PROPORTIONAL_INTAKE_VALUE;


public class ProportionalIntake extends CommandBase {
    private final Drivetrain drivetrain;
    private final Intake intake;

    public ProportionalIntake(Intake intake, Drivetrain drivetrain) {
        addRequirements(intake);
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        intake.setPosition(State.OPEN);
    }

    @Override
    public void execute() {
        double currentSpeed = Math.abs((drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2);
        intake.powerWheels(currentSpeed * PROPORTIONAL_INTAKE_VALUE.get() + INTAKE_CONSTANT_VALUE.get());
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
