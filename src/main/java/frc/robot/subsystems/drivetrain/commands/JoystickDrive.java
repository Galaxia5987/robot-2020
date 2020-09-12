package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.Constants.Drivetrain.JOYSTICK_MIN_THRESHOLD;
import static java.lang.Math.signum;

public class JoystickDrive extends CommandBase {
    Drivetrain drivetrain;
    public JoystickDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPower(0, 0);
    }

    @Override
    public void execute() {
        double rightPower = 0;
        double leftPower = 0;
        if (Math.abs(OI.getLeftStickForward()) > JOYSTICK_MIN_THRESHOLD)
            leftPower = OI.getLeftStickForward();
        if (Math.abs(OI.getRightStickForward()) > JOYSTICK_MIN_THRESHOLD)
            rightPower = OI.getRightStickForward();
        System.out.println(leftPower);
        drivetrain.setPower(quadraticCurveSpeed(leftPower), quadraticCurveSpeed(rightPower)); // TODO: change back to positive values
    }

    public double curveSpeed(double x) {
        double sign = signum(x);
        return sign / (1 + Math.exp(-7.2* (Math.abs(x) - 0.5)));
    }

    public double bellCurveSpeed(double x) {
        double a = 1;
        double b = 0.25;
        return Math.exp(-Math.pow((Math.abs(x)-a), 2)/b)*Math.signum(x);
    }

    public double quadraticCurveSpeed(double x){
        double b = 4, a = -1.1, c = 4.9, d = 2.3;
        return (b * Math.pow(x, 3) + a * Math.pow(x, 5) + c * x + d * Math.pow(x, 9)) / (a + b + c + d);
    }

}
