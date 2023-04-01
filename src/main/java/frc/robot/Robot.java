// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.config.DriveMap;

public class Robot extends TimedRobot {
    private RobotContainer mRobotContainer;
    private Command mAutoCommand;
    private UsbCamera cam;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        cam = CameraServer.startAutomaticCapture();
        cam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        // Create the robot's objects and subsystems and map user controls
        mRobotContainer = new RobotContainer();
        mRobotContainer.configureBindings();
    }

    @Override
    public void autonomousInit() {
        // mAutoCommand =
        // mRobotContainer.mAuto.getAutonomousCommand(mRobotContainer.mDrivetrain);
        DriveCommands.resetGyroComamand(mRobotContainer.mDrivetrain).schedule();
        mAutoCommand.schedule();
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
        if (mAutoCommand != null && !mAutoCommand.isFinished())
            mAutoCommand.end(true);

        // Kill the PID controllers used for trajectory following in autonomous
        // mRobotContainer.mDrivetrain.mAutoTranslationXController.close();
        // mRobotContainer.mDrivetrain.mAutoTranslationYController.close();
        // mRobotContainer.mDrivetrain.mAutoRotationController.close();
        // DriveCommands.resetGyroComamand(mRobotContainer.mDrivetrain).schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    }
}
