/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.TrajectoryLoader;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

import static frc.robot.RobotContainer.turret;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static final boolean debug = isDebug() && !DriverStation.getInstance().isFMSAttached();
    // The roboRIO has built-in pull up resistors, bridge signal and ground pins on Robot A DIO 0.
    public static boolean isRobotA = !new DigitalInput(0).get();
    public static boolean shootingManualMode = false;
    public static Compressor compressor = new Compressor();
    public static PowerDistributionPanel pdp = new PowerDistributionPanel();
    public static Timer robotTimer = new Timer();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * @return Robot in debug mode
     */
    private static boolean isDebug() {
        return false;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (Robot.isRobotA) {
            Utils.swapConstants(Constants.class, AConstants.class);
            Utils.swapConstants(Ports.class, APorts.class);
        }
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotTimer.reset();
        robotTimer.start();

        TrajectoryLoader.loadTrajectories();

        m_robotContainer = new RobotContainer();
        CustomDashboard.setAutonomousModes(m_robotContainer.getAutonomousModes());

//        compressor.stop();

        SmartDashboard.putBoolean("Robot A", isRobotA);
        SmartDashboard.putBoolean("Debug", debug);

//        PortForwarder.add(1181, "10.59.87.7", 1181);
        PortForwarder.add(8888, "10.59.87.7", 5800);
        PortForwarder.add(8887, "10.59.87.7", 22);

        //Disable live window for more loop time
        LiveWindow.setEnabled(false);
        LiveWindow.disableAllTelemetry();

        startCameraCapture();
    }

    public void startCameraCapture() {
        new Thread(() -> {
            UsbCamera camera = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(0);
            camera.setResolution(160, 120);
            camera.setFPS(30);
        }).start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        VisionModule.setLEDs(false);
    }

    @Override
    public void disabledPeriodic() {
        try {
            m_robotContainer.leds.disabledPeriodic();
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        turret.setPower(0);
        m_robotContainer.drivetrain.setBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_robotContainer.conveyor.resetBallCount();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        try {
            m_robotContainer.leds.autonomousPeriodic();
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.drivetrain.setBrake(false);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        try {
            m_robotContainer.leds.teleopPeriodic();
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
