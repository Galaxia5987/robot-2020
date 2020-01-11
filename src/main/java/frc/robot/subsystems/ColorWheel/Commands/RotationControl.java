package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String currentColor;
    private int currentIndex = 0;
    private double spinCounter = 0.125;

    public int indexOfColor(String color) {
        switch (color) {
            case ("Yellow"):
                return 0;
            case ("Red"):
                return 1;
            case ("Green"):
                return 2;
            case ("Blue"):
                return 3;
            default:
                return indexOfColor(currentColor);

        }
    }

    @Override
    public void initialize() {
        currentColor = colorWheel.getColorString();
        currentIndex = indexOfColor(currentColor);

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("spins", spinCounter);
        if (indexOfColor(colorWheel.getColorString()) == currentIndex + 1 || indexOfColor(colorWheel.getColorString()) == (currentIndex)%3) {
            currentColor = colorWheel.getColorString();
            currentIndex += 1;
            currentIndex %= 3;
            spinCounter += 0.125;
        }
        if (indexOfColor(colorWheel.getColorString()) == currentIndex - 1 || indexOfColor(colorWheel.getColorString()) == (currentIndex)%3) {
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
