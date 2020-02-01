package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.color_wheel.ColorWheel;

import java.util.function.Supplier;


/**
 * Rotates the control panel somewhere around 4 spins, does not mind an overshoot or an undershoot
 * stops at 4 spins
 */
public class ManualControl extends CommandBase {
    private ColorWheel colorWheel;

    private Supplier<Double> joystickInput = null;



    public ManualControl(ColorWheel colorWheel, Supplier<Double> joystickInput) {
        this.colorWheel = colorWheel;
        this.joystickInput = joystickInput;
    }

    @Override
    public void initialize() {
        colorWheel.setPower(Constants.ColorWheel.ROTATION_CONTROL_POWER);
    }

    @Override
    public void execute() {
            colorWheel.setPower(joystickInput.get());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

}
