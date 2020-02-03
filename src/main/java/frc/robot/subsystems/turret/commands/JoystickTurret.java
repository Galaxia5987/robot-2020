package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.Utils;

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
        turret.setAngle(Utils.constrain(position, MINIMUM_POSITION, MAXIMUM_POSITION));
    }

}
