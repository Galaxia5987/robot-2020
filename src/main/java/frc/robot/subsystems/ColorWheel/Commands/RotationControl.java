package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private double percentSpeed;
    private int sensorColorIndex;
    private int clockwiseColorIndex = 0;
    private int counterClockwiseIndex = 0;
    private double clockwiseSpins = 0;

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
        if (sensorColorIndex == (clockwiseColorIndex + 1) % 4){
            clockwiseSpins += 0.125;
            clockwiseColorIndex = sensorColorIndex;
        }
        if (sensorColorIndex == (counterClockwiseIndex - 1) % 4){
            clockwiseSpins -= 0.125;
            counterClockwiseIndex = sensorColorIndex;
        }

    }

    private void updateColorIndex(){
        try {
            sensorColorIndex = colorWheel.indexOfColor(colorWheel.getColorString());
        } catch (Exception ignored){

        }
    }

    @Override
    public boolean isFinished() {
        double counterClockWiseSpins = 0;
        return Math.max(clockwiseSpins, counterClockWiseSpins) >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }

}
