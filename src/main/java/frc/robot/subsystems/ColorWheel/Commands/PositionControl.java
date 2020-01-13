package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class PositionControl extends CommandBase {

    private final String[] colors = {"Yellow", "Red", "Green", "Blue"};
    private String currentColor;
    private int currentIndex = 0;
    private double spinCounter = 0.125;
    private String gameData;
    private String targetColor;

    public PositionControl(){
        gameData = DriverStation.getInstance().getGameSpecificMessage();
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
        return colorWheel.getColorString().equals(colors[colorWheel.indexOfColor(targetColor, targetColor)+2]);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setMotorSpeed(0);
    }
}
