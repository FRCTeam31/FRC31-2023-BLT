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
import frc.robot.subsystems.ArduinoSidecar.LEDMode;

public class Robot extends TimedRobot {
    private RobotContainer _robotContainer;
    private Command _autoCommand;
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

        // Create the robot's objects and subsystems and map user controls
        _robotContainer = new RobotContainer();
        _robotContainer.configureBindings();
        cam = CameraServer.startAutomaticCapture();
        cam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        _robotContainer.setLEDMode(LEDMode.SCAN_UP);
    }

    @Override
    public void disabledInit() {
        _robotContainer.setLEDMode(LEDMode.SCAN_UP);
    }

    @Override
    public void autonomousInit() {
        _robotContainer.setLEDMode(LEDMode.AUTO);
        _autoCommand = _robotContainer.getAutonomousCommand();
        _autoCommand.schedule();
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
        if (_autoCommand != null && !_autoCommand.isFinished())
            _autoCommand.end(true);

        // Kill the PID controllers used for trajectory following in autonomous
        // _robotContainer.mDrivetrain.mAutoTranslationXController.close();
        // _robotContainer.mDrivetrain.mAutoTranslationYController.close();
        // _robotCo$ntainer.mDrivetrain.mAutoRotationController.close();

        // _robotContainer.Drivetrain.resetGyro();
        _robotContainer.setLEDMode(LEDMode.TELEOP);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (DriverStation.isFMSAttached() &&
                DriverStation.getMatchTime() <= 30 &&
                _robotContainer.Sidecar.getMode() != LEDMode.SCAN_OUT) {
            _robotContainer.setLEDMode(LEDMode.SCAN_OUT);
        }
    }
}
