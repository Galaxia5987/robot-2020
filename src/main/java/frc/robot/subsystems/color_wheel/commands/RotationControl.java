package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private double percentSpeed;
    private int sensorColorIndex;
    private int clockwiseColorIndex = 0;
    private int counterClockwiseIndex = 0;
    private double clockwiseSpins = 0;
    private double counterClockwiseSpins = 0;

    public RotationControl(double percentSpeed) {
        this.percentSpeed = percentSpeed;
    }

    @Override
    public void initialize() {
        updateColorIndex();
        clockwiseColorIndex = sensorColorIndex;
        counterClockwiseIndex = sensorColorIndex;
        colorWheel.setMotorSpeed(percentSpeed);
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
        return Math.max(clockwiseSpins, counterClockwiseSpins) >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }

}
