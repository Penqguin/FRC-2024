// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.oi;

public class Robot extends TimedRobot {

    private drivetrain drive;
    private oi oi;

    @Override
    public void robotInit() {
    }

    @Override
    public void teleopInit() {
        drive = drivetrain.getInstance();
        oi = oi.getInstance();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        oi.periodic();
    }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    drive = drivetrain.getInstance();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
