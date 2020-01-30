package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.TURRET_JOYSTICK_SPEED;
import static frc.robot.RobotContainer.getRightXboxY;

public class JoystickTurret extends CommandBase {
    private final Turret turret;

    public JoystickTurret(Turret turret) { //This does not power the motors, it simply adjusts the target angle. Henceforth, there is no reason to interrupt the turning class.
        this.turret = turret;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double joystickInput = getRightXboxY();
        double offset = joystickInput * TURRET_JOYSTICK_SPEED;
        turret.setOffset(offset);
    }

}
