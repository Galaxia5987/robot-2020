/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.CustomDashboard;

public class AutoDelay extends CommandBase {
    protected Timer timer = new Timer();
    private double m_duration;

    @Override
    public void initialize() {
        m_duration = CustomDashboard.getAutoDelay();
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
