package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private String clockWiseCurrentColor = "Yellow";
    private String counterClockWiseCurrentColor = "Yellow";
    private double clockWiseSpins = 0;
    private double percentSpeed = 0;


    public RotationControl(double percentSpeed) {
        this.percentSpeed = percentSpeed;
    }

    @Override
    public void initialize() {
        clockWiseCurrentColor = colorWheel.getColorString();
        counterClockWiseCurrentColor = colorWheel.getColorString();
        colorWheel.setMotorSpeed(percentSpeed);
    }

    @Override
    public void execute() {
        String detectedColor = colorWheel.getColorString();
        if (colorWheel.indexOfColor(detectedColor) == colorWheel.indexOfColor(clockWiseCurrentColor) +1
                || colorWheel.indexOfColor(detectedColor) == colorWheel.indexOfColor(clockWiseCurrentColor) % 3){
            clockWiseSpins += 0.125;
            clockWiseCurrentColor = detectedColor;
        }
        if (colorWheel.indexOfColor(detectedColor) == colorWheel.indexOfColor(counterClockWiseCurrentColor) -1
                || colorWheel.indexOfColor(detectedColor)%3 == colorWheel.indexOfColor(clockWiseCurrentColor)){
            clockWiseSpins -= 0.125;
            counterClockWiseCurrentColor = detectedColor;
        }



    }

    @Override
    public boolean isFinished() {
        double counterClockWiseSpins = 0;
        return Math.max(clockWiseSpins, counterClockWiseSpins) >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }

}
