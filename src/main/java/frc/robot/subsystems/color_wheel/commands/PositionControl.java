package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.colorWheel;

public class PositionControl extends CommandBase {

    private final String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String currentColor;
    private int currentIndex = 0;
    private double spinCounter = 0.125;
    private String targetColor;

    public PositionControl(){
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0){
            switch (gameData.charAt(0)){
                case('Y'):
                    targetColor = "Yellow";
                case('R'):
                    targetColor = "Red";
                case('G'):
                    targetColor = "Green";
                case('B'):
                    targetColor = "Blue";
                default:
                    targetColor = "Unknown";
            }
        }
    }

    @Override
    public void initialize() {
        colorWheel.setMotorSpeed(0.2);
    }

    @Override
    public void execute() {

    }
    @Override
    public boolean isFinished() {
        return colorWheel.getColorString().equals(colors[colorWheel.indexOfColor(targetColor)+ Constants.ColorWheel.TILES_BEFORE_SENSOR]);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }
    }
