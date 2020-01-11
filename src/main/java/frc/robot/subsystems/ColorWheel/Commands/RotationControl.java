package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String currentColor;
    private int currentIndex = 0;
    private double spinCounter = 0.125;



    @Override
    public void initialize() {
        currentColor = colorWheel.getColorString();
        currentIndex = colorWheel.indexOfColor(currentColor, currentColor);

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("spins", spinCounter);
        if (colorWheel.indexOfColor(colorWheel.getColorString(), currentColor) == currentIndex + 1 ||
                colorWheel.indexOfColor(colorWheel.getColorString(), currentColor) == (currentIndex)%3) {
            currentColor = colorWheel.getColorString();
            currentIndex += 1;
            currentIndex %= 3;
            spinCounter += 0.125;
        }
        if (colorWheel.indexOfColor(colorWheel.getColorString(), currentColor) == currentIndex - 1
                || colorWheel.indexOfColor(colorWheel.getColorString(), currentColor) == (currentIndex)%3) {
            currentColor = colorWheel.getColorString();
            currentIndex -= 1;
            currentIndex %= 3;
            spinCounter -= 0.125;
        }

    }

    @Override
    public boolean isFinished() {
        return spinCounter >= 3 || spinCounter<= -3;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
