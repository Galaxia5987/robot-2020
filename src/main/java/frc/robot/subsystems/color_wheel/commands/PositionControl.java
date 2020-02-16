package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.color_wheel.ColorWheel;

import static frc.robot.Constants.ColorWheel.*;

/**
 * The commands uses the date from the fms to rotate the control panel to the given color
 */
public class PositionControl extends CommandBase {
    private String targetColor = null;
    private char targetColorChar;
    private int currentColor;
    private Timer endTimer = new Timer(); // Used to make sure we don't overshoot over the wanted color.
    private ColorWheel colorWheel;


    public PositionControl(ColorWheel colorWheel) {
        addRequirements(colorWheel);
        this.colorWheel = colorWheel;
    }

    public PositionControl(ColorWheel colorWheel, String targetColor) {
        addRequirements(colorWheel);
        this.colorWheel = colorWheel;
        this.targetColor = targetColor;
    }

    private char getColor() {
        if (targetColor != null)
            return targetColor.toUpperCase().charAt(0);
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            gameData = gameData.toUpperCase();
            if (gameData.charAt(0) == 'Y' || gameData.charAt(0) == 'R' || gameData.charAt(0) == 'G' || gameData.charAt(0) == 'B')
                return gameData.charAt(0);
        }
        this.cancel();
        return ' ';
    }

    @Override
    public void initialize() {
        this.targetColorChar = getColor();
        endTimer.reset();
    }

    @Override
    public void execute() {
        currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
        int distanceFromTarget = Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(targetColorChar)) - TILES_BEFORE_SENSOR, 4);
        colorWheel.setPower(Math.IEEEremainder(distanceFromTarget, 4) * kP.get());
        if (distanceFromTarget == 0 && endTimer.get() == 0)
            endTimer.start();
        else if (distanceFromTarget != 0)
            endTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return (endTimer.get() > POSITION_CONTROL_TIMER && Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(targetColorChar)) - TILES_BEFORE_SENSOR, 4) == 0);
    }

    @Override
    public void end(boolean interrupted) {
        endTimer.stop();
        colorWheel.setPower(0);
    }
}
