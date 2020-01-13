package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Turret.*;
import static frc.robot.RobotContainer.*;


public class JoystickTurret extends CommandBase {
    private UnitModel unitModel = new UnitModel(TICKS_PER_DEGREE);

    public JoystickTurret() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double joystickInput = RobotContainer.m_robotContainer.getXboxY();
        double position = turret.getEncoderPosition() + joystickInput * TURRET_JOYSTICK_SPEED;
        if (position < MINIMUM_POSITION && position > MAXIMUM_POSITION)
            turret.setPosition(position);
    }

}
