package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.Constants.Drivetrain.JOYSTICK_MIN_THRESHOLD;

public class JoystickDrive extends CommandBase {
    Drivetrain drivetrain;
    public JoystickDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double rightPower = 0;
        double leftPower = 0;
        if (Math.abs(OI.getLeftStickForward()) > JOYSTICK_MIN_THRESHOLD)
           leftPower =  OI.getLeftStickForward() * 0.7;
        if (Math.abs(OI.getRightStickForward()) > JOYSTICK_MIN_THRESHOLD)
            rightPower = OI.getRightStickForward() * 0.7;
        drivetrain.setPower(leftPower, rightPower);
    }

}
