package frc.robot.subsystems.color_wheel.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.Constants.ColorWheel.*;
import static frc.robot.RobotContainer.colorWheel;

public class PositionControl extends CommandBase {

    private final String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String targetColor;
    private int currentColor;
    private Timer endTimer = new Timer();

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
        colorWheel.setNeutralMode(NeutralMode.Coast);
        endTimer.reset();
    }

    @Override
    public void execute() {
        int distanceFromTarget = Math.abs(currentColor - colorWheel.indexOfColor(targetColor));
        if (distanceFromTarget < REVERSE_TILE_THRESHOLD)
            colorWheel.setMotorSpeed(POSITION_SPEED * (distanceFromTarget * kP));
        else
            colorWheel.setMotorSpeed(-POSITION_SPEED * (distanceFromTarget * kP));
        if (distanceFromTarget == 0){
            endTimer.start();
        }
    }

    @Override
    public boolean isFinished() {
        try {
            return targetColor.equals(colors[colorWheel.indexOfColor(colorWheel.getColorString()) + Constants.ColorWheel.TILES_BEFORE_SENSOR])
                    && endTimer.get() > 4;
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        endTimer.stop();
        colorWheel.setMotorSpeed(0);
        colorWheel.setNeutralMode(NeutralMode.Brake);
    }

    public void getCurrentColor() {
        try {
            currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
        } catch (Exception ignored) {

        }
    }
}
