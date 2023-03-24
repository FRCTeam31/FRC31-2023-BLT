package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.ControlsMap;
import frc.robot.config.WristMap;
import frc.robot.models.IntakeDirection;

public class RobotContainer implements Sendable {
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
            mForearm = new Forearm();
            SmartDashboard.putData(mForearm);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Forearm subsystem", true);
            return;
        }

        try {
            mWrist = new Wrist();
            SmartDashboard.putData(mWrist);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
            return;
        }

        try {
            mFrontCamera = new FrontCamera();
            SmartDashboard.putData(mFrontCamera);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize FrontCamera", true);
            return;
        }
    }

    public void configureBindings() {
        // Set up controllers
        mDriverController = new CommandJoystick(ControlsMap.DRIVER_PORT);
        mOperatorController = new CommandJoystick(ControlsMap.OPERATOR_PORT);

        // Default commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDrivetrain,
                () -> mDriverController.getRawAxis(
                        ControlsMap.RIGHT_STICK_Y),
                () -> mDriverController.getRawAxis(
                        ControlsMap.RIGHT_STICK_X),
                () -> mDriverController.getRawAxis(ControlsMap.LEFT_STICK_X),
                mDrivetrain.mSwerveModules,
                true));
        mWrist.setDefaultCommand(WristCommands.runIntake(mWrist,
                () -> mOperatorController.getRawAxis(ControlsMap.LEFT_TRIGGER) > WristMap.triggerDeadBand,
                () -> mOperatorController.getRawAxis(ControlsMap.RIGHT_TRIGGER) > WristMap.triggerDeadBand));
        mForearm.setDefaultCommand(ForearmCommands.controlWithJoystick(mForearm,
                () -> -mOperatorController.getRawAxis(ControlsMap.RIGHT_STICK_Y)));

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
        // mOperatorController.button(ControlsMap.LB)
        // .whileTrue(EndEffectorCommands.raiseEffectorManually(mShoulder, // While LB
        // is held, control the arm
        // // speed with the left stick Y axis
        // () -> mForearm.getMinSoftLimitReached(),
        // () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
        // .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is
        // released, set the shoulder

        mOperatorController.button(ControlsMap.LB)
                .whileTrue(ShoulderCommands.runWithJoystick(mShoulder, // While LB is held, control the arm
                                                                       // speed with the left stick Y axis
                        () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
                .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is released, set the shoulder

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
        // mDrivetrain.mAutoTranslationXController = new PIDController(
        // DriveMap.kAutoTranslationPID_kP,
        // DriveMap.kAutoTranslationPID_kI,
        // DriveMap.kAutoTranslationPID_kD);
        // mDrivetrain.mAutoTranslationYController = new PIDController(
        // DriveMap.kAutoTranslationPID_kP,
        // DriveMap.kAutoTranslationPID_kI,
        // DriveMap.kAutoTranslationPID_kD);
        // mDrivetrain.mAutoRotationController = new PIDController(
        // DriveMap.kAutoRotationPID_kP,
        // DriveMap.kAutoRotationPID_kI,
        // DriveMap.kAutoRotationPID_kD);

        // SmartDashboard.putData("Auto TranslationX PID",
        // mDrivetrain.mAutoTranslationXController);
        // SmartDashboard.putData("Auto TranslationY PID",
        // mDrivetrain.mAutoTranslationYController);
        // SmartDashboard.putData("Auto Rotation PID",
        // mDrivetrain.mAutoRotationController);

        // return new SequentialCommandGroup(
        // AutoCommands.translateYMeters(mDrivetrain, 0.48,
        // driveSpeed), // Push cube forward
        // AutoCommands.translateYMeters(mDrivetrain, -2,
        // driveSpeed) // Back up onto the ramp
        // );
        return new SequentialCommandGroup(
                Commands.run(() -> mDrivetrain.drive(0, 0.75, 0, true), mDrivetrain).withTimeout(3),
                Commands.run(() -> mDrivetrain.drive(0, 0, 0, true), mDrivetrain)

        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub

    }
}
