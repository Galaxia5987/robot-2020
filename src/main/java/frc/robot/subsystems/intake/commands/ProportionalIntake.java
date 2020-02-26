package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.Constants.Intake.PROPORTIONAL_INTAKE_VALUE;


public class ProportionalIntake extends CommandBase {
    private final Drivetrain drivetrain;
    private final Intake intake;

    public ProportionalIntake(Intake intake, Drivetrain drivetrain) {
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        double currentSpeed = (drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2;
        intake.powerWheels(currentSpeed * PROPORTIONAL_INTAKE_VALUE.get());
    }

    @Override
    public void end(boolean interrupted) {
        intake.powerWheels(0);
    }
}
