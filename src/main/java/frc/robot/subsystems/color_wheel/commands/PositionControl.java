package frc.robot.subsystems.color_wheel.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.color_wheel.ColorWheel;

import java.util.function.Supplier;

import static frc.robot.Constants.ColorWheel.*;

/**
 * The commands uses the date from the fms to rotate the control panel to the given color
 */
public class PositionControl extends CommandBase {
    private char FMSData;
    private int currentColor;
    private Timer endTimer = new Timer(); // Used to make sure we don't overshoot over the wanted color.
    private ColorWheel colorWheel;
    private Supplier<Double> joystickInput = OI::getRightXboxX;

    public PositionControl(ColorWheel colorWheel) {
        this.colorWheel = colorWheel;
    }

    @Override
    public void initialize() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            if(gameData.charAt(0) == 'Y' || gameData.charAt(0) == 'R' || gameData.charAt(0) == 'G' || gameData.charAt(0) == 'B')
                FMSData = gameData.charAt(0);
            else
                this.cancel();
        }
        else
            this.cancel();

        endTimer.reset();
    }

    @Override
    public void execute() {
        try {
            currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
            int distanceFromTarget = Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(FMSData))  - TILES_BEFORE_SENSOR, 4);
            if (joystickInput.get() > 0.1)
                colorWheel.setManual(true);
            if (!colorWheel.isManual())
                colorWheel.setPower(POSITION_CONTROL_POWER * (Math.IEEEremainder(distanceFromTarget, 4) * kP));
            else
                colorWheel.setPower(joystickInput.get());
            if (distanceFromTarget == 0 && endTimer.get() == 0)
                endTimer.start();
            else if (distanceFromTarget != 0)
                endTimer.reset();

        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        try {
            return (endTimer.get() > POSITION_CONTROL_TIMER && Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(FMSData)) - TILES_BEFORE_SENSOR, 4) == 0);
        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        endTimer.stop();
        colorWheel.setPower(0);
    }
}
