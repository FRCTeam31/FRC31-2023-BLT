package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ArduinoSidecar.LEDMode;
import frc.robot.subsystems.Wrist.WristMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.config.ControlsMap;

public class RobotContainer implements Sendable {
    public CommandJoystick mDriverController;
    public CommandJoystick mOperatorController;
    public Drivetrain mDrivetrain;
    public Shoulder mShoulder;
    public Wrist mWrist;
    public Forearm mForearm;
    public ArduinoSidecar mSidecar;
    public PowerDistribution mPdp;
    // public FrontCamera mFrontCamera;

    public RobotContainer() {
        try {
            mPdp = new PowerDistribution();
            SmartDashboard.putData(mPdp);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Drivetrain subsystem", true);
            return;
        }

        try {
            Drivetrain = new Drivetrain();
            Drivetrain.register();

            SmartDashboard.putData(Drivetrain);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Drivetrain subsystem", true);
            return;
        }

        try {
            mShoulder = new Shoulder();
            SmartDashboard.putData(mShoulder);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Shoulder subsystem", true);
            return;
        }

        try {
            Forearm = new Forearm();
            Forearm.register();

            SmartDashboard.putData(Forearm);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Forearm subsystem", true);
            return;
        }

        try {
            Wrist = new Wrist();
            Wrist.register();

            SmartDashboard.putData(Wrist);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
            return;
        }

        try {
            Sidecar = new ArduinoSidecar();
            Sidecar.register();

            SmartDashboard.putData(Sidecar);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
            return;
        }

        // try {
        // mFrontCamera = new FrontCamera();
        // SmartDashboard.putData(mFrontCamera);
        // } catch (Exception e) {
        // DriverStation.reportError("Failed to initalize FrontCamera", true);
        // return;
        // }
    }

    public void configureBindings() {
        // Set up controllers
        DriverController = new CommandJoystick(ControlsMap.DRIVER_PORT);
        OperatorController = new CommandJoystick(ControlsMap.OPERATOR_PORT);

        // Default commands
        Drivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(Drivetrain,
                () -> DriverController.getRawAxis(
                        ControlsMap.RIGHT_STICK_Y),
                () -> DriverController.getRawAxis(
                        ControlsMap.RIGHT_STICK_X),
                () -> DriverController.getRawAxis(ControlsMap.LEFT_STICK_X),
                Drivetrain.mSwerveModules,
                true));

        Wrist.setDefaultCommand(WristCommands.runIntake(Wrist,
                () -> OperatorController.getRawAxis(ControlsMap.LEFT_TRIGGER) > WristMap.kTriggerDeadband,
                () -> OperatorController.getRawAxis(ControlsMap.RIGHT_TRIGGER)));

        // Drive commands
        DriverController.button(ControlsMap.X).onTrue(Commands.runOnce(() -> Drivetrain.resetGyro(), Drivetrain));
        DriverController.button(ControlsMap.Y).onTrue(DriveCommands.toggleShifter(Drivetrain));

        // Shoulder commands
        mOperatorController.button(ControlsMap.RB)
                .onTrue(Commands.runOnce(() -> mForearm.resetEncoderOffset(), mForearm));
        mOperatorController.button(ControlsMap.LOGO_LEFT)
                .onTrue(ForearmCommands.home(mForearm));
        mOperatorController.button(ControlsMap.LOGO_RIGHT)
                .onTrue(ShoulderCommands.togglePID(mShoulder));
        mOperatorController.button(ControlsMap.Y)
                .onTrue(ShoulderCommands.setAngle(mShoulder,
                        Shoulder.Map.kTopRowAngle));
        mOperatorController.button(ControlsMap.B)
                .onTrue(ShoulderCommands.setAngle(mShoulder,
                        Shoulder.Map.kMiddleRowAngle));
        mOperatorController.button(ControlsMap.A)
                .onTrue(ShoulderCommands.setAngle(mShoulder,
                        Shoulder.Map.kGroundAngle));
        // mOperatorController.button(ControlsMap.LB)
        // .whileTrue(EndEffectorCommands.raiseEffectorManually(mShoulder, // While LB
        // is held, control the arm
        // // speed with the left stick Y axis
        // () -> mForearm.getMinSoftLimitReached(),
        // () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
        // .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is
        // released, set the shoulder

        // Forearm commands
        OperatorController.button(ControlsMap.RB).onTrue(ForearmCommands.extendForearm(Forearm));
        OperatorController.button(ControlsMap.LB).onTrue(ForearmCommands.retractForearm(Forearm));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                DriveCommands.SetWheelAnglesCommand(Drivetrain, Rotation2d.fromDegrees(180)),
                DriveCommands.resetOdometry(Drivetrain, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))),
                DriveCommands.resetGyroCommand(Drivetrain),
                AutonomousCommands.getAutonomousCommand(Drivetrain, Shoulder, Forearm, Wrist));
    }

    public void setLEDMode(LEDMode mode) {
        Sidecar.setMode(mode);
    }

    );

    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

}
