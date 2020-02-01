package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.MAXIMUM_POSITION;
import static frc.robot.Constants.Turret.MINIMUM_POSITION;
import static frc.robot.RobotContainer.TURRET_JOYSTICK_SPEED;
import static frc.robot.Constants.Turret.*;


public class JoystickTurret extends CommandBase {
    private static Turret turret;

    public JoystickTurret(Turret turret) {
        JoystickTurret.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double joystickInput = OI.getXboxY();
        double position = turret.getAngle() + joystickInput * TURRET_JOYSTICK_SPEED;
        if (position >= MINIMUM_POSITION && position <= MAXIMUM_POSITION)
            turret.setAngle(position);
    }

}
