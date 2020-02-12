package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.Drivetrain;

import static frc.robot.Constants.Drivetrain.JOYSTICK_MIN_OUTPUT;

public class JoystickDrive extends CommandBase {
    Drivetrain drivetrain;
    public JoystickDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (isValid())
            drivetrain.setPower(OI.getLeftStickForward() * 0.7, OI.getRightStickForward() * 0.7);
    }

    /**
     * @return if the output of the joystick is greater than the minimum output
     */
    private boolean isValid(){
        return OI.getLeftStickForward() * 0.7 > JOYSTICK_MIN_OUTPUT;
    }
}
