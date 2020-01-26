/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.*;
import frc.robot.utilities.StickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static final int rightYStick = 5;
    public static final double TURRET_JOYSTICK_SPEED = 1; //Coefficient of the joystick value per degree.
    private static final Intake intake = new Intake();
    private static final Turret turret = new Turret();
    // The robot's subsystems and commands are defined here...
    private static Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final XboxController xbox = new XboxController(2);
    private final JoystickButton a = new JoystickButton(xbox, 1);
    private final JoystickButton b = new JoystickButton(xbox, 2);
    private final JoystickButton rs = new JoystickButton(xbox, 10);
    private final StickButton rightY = new StickButton(xbox, 5, 0.1);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        turret.setDefaultCommand(new TurretSwitching(turret));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        a.whenPressed(new TurnTurret(turret, 45));
        b.whenPressed(new CenterTurret(turret));
        rightY.whileHeld(new JoystickTurret(turret));
        rs.whenPressed(new ResetDelta(turret));
    }

    public double getXboxY() {
        return xbox.getRawAxis(rightYStick);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
