package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.color_wheel.ColorWheel;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private int sensorColorIndex;
    private int clockwiseColorIndex = 0;
    private int counterClockwiseIndex = 0;
    private double clockwiseSpins = 0;
    private double counterClockwiseSpins = 0;
    private ColorWheel colorWheel;


    public RotationControl(ColorWheel colorWheel) {
        this.colorWheel = colorWheel;
    }

    @Override
    public void initialize() {
        updateColorIndex();
        clockwiseColorIndex = sensorColorIndex;
        counterClockwiseIndex = sensorColorIndex;
        colorWheel.setPower(Constants.ColorWheel.ROTATION_CONTROL_POWER);
    }

    @Override
    public void execute() {
        updateColorIndex();
        if (sensorColorIndex == (clockwiseColorIndex + 1) % 4) {
            clockwiseSpins += 0.125;
            clockwiseColorIndex = sensorColorIndex;
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
