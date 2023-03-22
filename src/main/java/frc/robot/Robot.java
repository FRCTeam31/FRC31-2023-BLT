// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;

public class Robot extends TimedRobot {
    private RobotContainer mRobotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        mRobotContainer = new RobotContainer();
        mRobotContainer.configureBindings();
    }

    @Override
    public void autonomousInit() {
        // var mAutoCommand = mRobotContainer.getAutonomousCommand();
        // DriveCommands.resetGyroComamand(mRobotContainer.mDrivetrain);
        // mAutoCommand.schedule();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // DriveCommands.resetGyroComamand(mRobotContainer.mDrivetrain).schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }
}
