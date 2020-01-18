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
    private char FMSData;
    private int currentColor;
    private Timer endTimer = new Timer();

    public PositionControl() {

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

        colorWheel.setNeutralMode(NeutralMode.Coast);
        endTimer.reset();
    }

    @Override
    public void execute() {
        currentColor = colorWheel.indexOfColor(colorWheel.getColorString());
        int distanceFromTarget = Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(FMSData))  - TILES_BEFORE_SENSOR, 4);
        if (distanceFromTarget < 3)
            colorWheel.setMotorSpeed(POSITION_CONTROL_SPEED * (distanceFromTarget * kP));
        else
            colorWheel.setMotorSpeed(-POSITION_CONTROL_SPEED * (distanceFromTarget * kP));
        if (distanceFromTarget == 0 && endTimer.get() != 0)
            endTimer.start();
        else if(distanceFromTarget != 0)
            endTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return endTimer.get() > POSITION_CONTROL_TIMER && Math.floorMod(currentColor - colorWheel.indexOfColor(Character.toString(FMSData))  - TILES_BEFORE_SENSOR, 4) == 0;
    }

    @Override
    public void end(boolean interrupted) {
        endTimer.stop();
        colorWheel.setMotorSpeed(0);
        colorWheel.setNeutralMode(NeutralMode.Brake);
    }
}
