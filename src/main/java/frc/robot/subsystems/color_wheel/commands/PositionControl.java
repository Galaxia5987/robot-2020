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
    private char targetColorChar;
    private Integer currentColor;
    private Timer endTimer = new Timer(); // Used to make sure we don't overshoot over the wanted color.
    private ColorWheel colorWheel;


    public PositionControl(ColorWheel colorWheel) {
        addRequirements(colorWheel);
        this.colorWheel = colorWheel;
    }

    private char getColor() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData != null) {
            if (gameData.length() > 0) {
                gameData = gameData.toUpperCase();
                if ("RGBY".contains(gameData))
                    return gameData.charAt(0);
            }
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
        colorWheel.updateSensor();
        currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
        Integer targetIndex = colorWheel.indexOfColor(Character.toString(targetColorChar));
        if (currentColor == null || targetIndex == null) {
            this.cancel();
            return;
        }
        int distanceFromTarget = Math.floorMod(currentColor - targetIndex - TILES_BEFORE_SENSOR, 4);
        switch (distanceFromTarget) {
            case (2):
                colorWheel.setPower(kP);
                break;
            case (1):
                colorWheel.setPower(kI);
                break;
            case (3):
                colorWheel.setPower(-kI);
                break;
            default:
                colorWheel.setPower(0);
        }
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
