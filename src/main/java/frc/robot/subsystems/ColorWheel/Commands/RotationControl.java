package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private String currentColor;
    private int currentIndex = 0;
    private double spinCounter = 0;
    private double relativeCounter = 0;
    private String relativeColor = "";
    private String lastRelativeColor = "";
    private double percentSpeed = 0;


    public RotationControl(double percetSpeed) {
        percentSpeed = percetSpeed;
    }

    @Override
    public void initialize() {
        currentColor = colorWheel.getColorString();
        currentIndex = colorWheel.indexOfColor(currentColor);
        colorWheel.setMotorSpeed(percentSpeed);

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("spins", spinCounter);
        relativeColor = colorWheel.getColorString();
        if (colorWheel.indexOfColor(lastRelativeColor) == colorWheel.indexOfColor(relativeColor)+1
                || colorWheel.indexOfColor(lastRelativeColor) == colorWheel.indexOfColor(relativeColor)%3)
            relativeCounter += 0.125;
        else
            relativeCounter = 0;
        lastRelativeColor = relativeColor;
        if (colorWheel.indexOfColor(colorWheel.getColorString()) == currentIndex + 1 ||
                colorWheel.indexOfColor(colorWheel.getColorString()) == (currentIndex) % 3) {
            currentColor = colorWheel.getColorString();
            currentIndex += 1;
            currentIndex %= 3;
            spinCounter += 0.125;
        }
        if (colorWheel.indexOfColor(colorWheel.getColorString()) == currentIndex - 1
                || colorWheel.indexOfColor(colorWheel.getColorString()) == (currentIndex) % 3) {
            currentColor = colorWheel.getColorString();
            currentIndex -= 1;
            currentIndex %= 3;
            spinCounter -= 0.125;
        }

    }

    @Override
    public boolean isFinished() {
        if (relativeCounter > 0.375)
            return relativeCounter >= 3 || relativeCounter <= -3;
        else
            return spinCounter >= 3 || spinCounter <= -3;
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }

}
