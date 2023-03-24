package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.ControlsMap;
import frc.robot.config.DriveMap;
import frc.robot.models.IntakeDirection;

public class RobotContainer {
    public CommandJoystick mDriverController;
    public CommandJoystick mOperatorController;
    public Drivetrain mDrivetrain;
    public Shoulder mShoulder;
    public Wrist mWrist;
    public Forearm mForearm;
    public ArduinoSidecar mSidecar;
    public FrontCamera mFrontCamera;

    public RobotContainer() {
        try {
            mDrivetrain = new Drivetrain();
            SmartDashboard.putData(mDrivetrain);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Drivetrain subsystem", true);
        }

        try {
            mShoulder = new Shoulder();
            SmartDashboard.putData(mShoulder);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Shoulder subsystem", true);
        }

        try {
            mForearm = new Forearm();
            SmartDashboard.putData(mForearm);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Forearm subsystem", true);
        }

        try {
            mWrist = new Wrist();
            SmartDashboard.putData(mWrist);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
        }

        try {
            mFrontCamera = new FrontCamera();
            SmartDashboard.putData(mFrontCamera);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize FrontCamera", true);
        }
    }

    public void configureBindings() {
        // Set up controllers
        mDriverController = new CommandJoystick(ControlsMap.DRIVER_PORT);
        mOperatorController = new CommandJoystick(ControlsMap.OPERATOR_PORT);

        // Default commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDrivetrain,
                () -> mDriverController.getRawAxis(
                        ControlsMap.LEFT_STICK_Y),
                () -> mDriverController.getRawAxis(
                        ControlsMap.LEFT_STICK_X),
                () -> mDriverController.getRawAxis(ControlsMap.RIGHT_STICK_X), mDrivetrain.mSwerveModules, true));
        mWrist.setDefaultCommand(WristCommands.runIntake(mWrist, mOperatorController));
        mForearm.setDefaultCommand(ForearmCommands.controlWithJoystick(mForearm,
                () -> mOperatorController.getRawAxis(ControlsMap.RIGHT_STICK_Y)));

        // Drive commands
        mDriverController.button(ControlsMap.X).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));
        mDriverController.button(ControlsMap.Y).onTrue(DriveCommands.toggleShifter(mDrivetrain));

        // Shoulder commands
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
        mOperatorController.button(ControlsMap.LB)
                .whileTrue(EndEffectorCommands.raiseEffectorManually(mShoulder, // While LB is held, control the arm
                                                                                // speed with the left stick Y axis
                        () -> mForearm.getMinSoftLimitReached(),
                        () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
                .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is released, set the shoulder
                                                                        // setpoint to the current angle

        // Wrist commands
        mOperatorController.pov(ControlsMap.up)
                .onTrue(WristCommands.setWristAngle(mWrist, IntakeDirection.kCone));

        mOperatorController.pov(ControlsMap.down)
                .onTrue(WristCommands.setWristAngle(mWrist, IntakeDirection.kCube));

        // Auto testing commands, only enabled when we're not on the field
        if (!DriverStation.isFMSAttached()) {
            var autoDriveSpeed = 1 / 4d;
            mDriverController.button(ControlsMap.LB)
                    .onTrue(getAutonomousCommand(autoDriveSpeed));
        }
    }

    public SequentialCommandGroup getAutonomousCommand(double driveSpeed) {
        mDrivetrain.mAutoTranslationXController = new PIDController(
                DriveMap.kAutoTranslationPID_kP,
                DriveMap.kAutoTranslationPID_kI,
                DriveMap.kAutoTranslationPID_kD);
        mDrivetrain.mAutoTranslationYController = new PIDController(
                DriveMap.kAutoTranslationPID_kP,
                DriveMap.kAutoTranslationPID_kI,
                DriveMap.kAutoTranslationPID_kD);
        mDrivetrain.mAutoRotationController = new PIDController(
                DriveMap.kAutoRotationPID_kP,
                DriveMap.kAutoRotationPID_kI,
                DriveMap.kAutoRotationPID_kD);

        SmartDashboard.putData("Auto TranslationX PID", mDrivetrain.mAutoTranslationXController);
        SmartDashboard.putData("Auto TranslationY PID", mDrivetrain.mAutoTranslationYController);
        SmartDashboard.putData("Auto Rotation PID", mDrivetrain.mAutoRotationController);

        return new SequentialCommandGroup(
                AutoCommands.translateYMeters(mDrivetrain, 0.48,
                        driveSpeed), // Push cube forward
                AutoCommands.translateYMeters(mDrivetrain, -2,
                        driveSpeed) // Back up onto the ramp
        );
    }
}
