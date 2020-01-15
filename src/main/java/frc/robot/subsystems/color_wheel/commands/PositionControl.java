package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.colorWheel;

public class PositionControl extends CommandBase {

    private final String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String targetColor;

    public PositionControl() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case ('Y'):
                    targetColor = "Yellow";
                case ('R'):
                    targetColor = "Red";
                case ('G'):
                    targetColor = "Green";
                case ('B'):
                    targetColor = "Blue";
                default:
                    targetColor = "Unknown";
            }
        }
    }

    @Override
    public void initialize() {
        colorWheel.setMotorSpeed(Constants.ColorWheel.POSITION_SPEED);
    }

    @Override
    public void execute() {
        int distanceFromTarget = Math.abs(currentColor - colorWheel.indexOfColor(targetColor));
        if (distanceFromTarget < REVERSE_TILE_THRESHOLD)
            colorWheel.setMotorSpeed(POSITION_SPEED * (distanceFromTarget * kP));
        else
            colorWheel.setMotorSpeed(-POSITION_SPEED * (distanceFromTarget * kP));
    }

    @Override
    public boolean isFinished() {
        try {
            return targetColor.equals(colors[colorWheel.indexOfColor(colorWheel.getColorString()) + Constants.ColorWheel.TILES_BEFORE_SENSOR]);
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    public void getCurrentColor() {
        try {
            currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
        } catch (Exception ignored) {

    }
}
}
