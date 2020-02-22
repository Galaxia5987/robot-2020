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
import frc.robot.utilities.TrajectoryLoader;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;
import org.eclipse.jetty.util.thread.Scheduler;

import static frc.robot.RobotContainer.navx;


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
    private Command m_autonomousCommand;
    public static Timer robotTimer = new Timer();
    private RobotContainer m_robotContainer;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;
    /**
     * @return Robot in debug mode
     */
    private static boolean isDebug() {
        return true;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if(!Robot.isRobotA) {
            Utils.swapConstants(Constants.class, BConstants.class);
            Utils.swapConstants(Ports.class, BPorts.class);
        }
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        TrajectoryLoader.loadTrajectories();
        robotTimer.reset();
        robotTimer.start();
        m_robotContainer = new RobotContainer();
//        compressor.stop();
        SmartDashboard.putBoolean("Robot A", isRobotA);
        SmartDashboard.putBoolean("Debug", debug);
        SmartDashboard.putData("running command", CommandScheduler.getInstance());


        SmartDashboard.putData(pdp);
        SmartDashboard.putData(navx);

        //Disable live window for more loop time
        LiveWindow.setEnabled(false);
        LiveWindow.disableAllTelemetry();

        startCameraCapture();
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(22);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
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
        rainbow();
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
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
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.drivetrain.setBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
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
