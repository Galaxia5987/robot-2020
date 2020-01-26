package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.color_wheel.ColorWheel;


/**
 * Rotates the control panel somewhere around 4 spins, does not mind an overshoot or an undershoot
 * stops at 4 spins
 */
public class RotationControl extends CommandBase {
    private int sensorColorIndex;
    private int clockwiseIndex = 0;//Marks the index of the current color when looking for a clockwise pov
    private int counterClockwiseIndex = 0;//Marks the index of the current color when looking for a counterclockwise pov
    private double clockwiseSpins = 0;//Counts the clockwise spins of the control panel
    private double counterClockwiseSpins = 0;//Counts the counterclockwise spins of the control panel
    private ColorWheel colorWheel;



    public RotationControl(ColorWheel colorWheel) {
        this.colorWheel = colorWheel;
    }

    @Override
    public void initialize() {
        updateColorIndex();
        clockwiseIndex = sensorColorIndex;
        counterClockwiseIndex = sensorColorIndex;
        colorWheel.setPower(Constants.ColorWheel.ROTATION_CONTROL_POWER);
    }

    @Override
    public void execute() {
        updateColorIndex();
        /*
        this block of code looks at the order of the colors and checks whether the wheel is moving clockwise or counterclockwise
        and counts the amount of spis to each direction accordingly
         */
        if (sensorColorIndex == (clockwiseIndex + 1) % 4) {
            clockwiseSpins += 0.125;
            clockwiseIndex = sensorColorIndex;
        }
        if (sensorColorIndex == Math.floorMod((counterClockwiseIndex - 1) , 4)) {
            counterClockwiseSpins += 0.125;
            counterClockwiseIndex = sensorColorIndex;
        }

    }

    private void updateColorIndex() {
        try {
            sensorColorIndex = colorWheel.indexOfColor(colorWheel.getColorString());
        } catch (Exception ignored) {

        }

    }

    @Override
    public boolean isFinished() {
        return Math.max(clockwiseSpins, counterClockwiseSpins) >= 4;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

}
