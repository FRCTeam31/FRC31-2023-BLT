package frc.robot;

import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.robot.config.AutoMap;
import frc.robot.config.ControlsMap;

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
        mDrivetrain = new Drivetrain();
        SmartDashboard.putData(mDrivetrain);

        mShoulder = new Shoulder();
        SmartDashboard.putData(mShoulder);

        mForearm = new Forearm();
        SmartDashboard.putData(mForearm);

        mWrist = new Wrist();
        SmartDashboard.putData(mWrist);

        mFrontCamera = new FrontCamera();
        SmartDashboard.putData(mFrontCamera);
    }

    public void configureBindings() {
        mDriverController = new CommandJoystick(ControlsMap.DRIVER_PORT);
        mOperatorController = new CommandJoystick(ControlsMap.OPERATOR_PORT);

        // Drive commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDrivetrain,
                () -> mDriverController.getRawAxis(
                        ControlsMap.LEFT_STICK_Y),
                () -> mDriverController.getRawAxis(
                        ControlsMap.LEFT_STICK_X),
                () -> mDriverController.getRawAxis(ControlsMap.RIGHT_STICK_X), mDrivetrain.mSwerveModules, true));
        mDriverController.button(ControlsMap.X).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));
        mDriverController.button(ControlsMap.Y).onTrue(DriveCommands.toggleShifter(mDrivetrain));

        // Shoulder commands
        mOperatorController.button(ControlsMap.LOGO_RIGHT).onTrue(ShoulderCommands.togglePID(mShoulder));
        mOperatorController.button(ControlsMap.Y)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kTopRowAngle));
        mOperatorController.button(ControlsMap.B)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kMiddleRowAngle));
        mOperatorController.button(ControlsMap.A)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kGroundAngle));

        mOperatorController.button(ControlsMap.LB)
                .whileTrue(EndEffectorCommands.raiseEffectorManually(mShoulder,
                        () -> mForearm.getMinSoftLimitReached(),
                        () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)));
        // mOperatorController.button(ControlsMap.LB)
        // .whileTrue(ShoulderCommands.disablePidAndRunManually(mShoulder, // While LB
        // is held, control the arm
        // // speed with the left stick Y axis
        // () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
        // .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is
        // released, set the shoulder
        // // setpoint to the current angle

        // Wrist commands
        mWrist.setDefaultCommand(WristCommands.runIntake(mWrist, mOperatorController));
        mOperatorController.pov(ControlsMap.up)
                .onTrue(WristCommands.setWristCommand(mWrist, true));

        mOperatorController.pov(ControlsMap.down)
                .onTrue(WristCommands.setWristCommand(mWrist, false));

        // Forearm commands
        mForearm.setDefaultCommand(ForearmCommands.controlWithJoystick(mForearm,
                () -> mOperatorController.getRawAxis(ControlsMap.RIGHT_STICK_Y)));
    }

    public SequentialCommandGroup getAutonomousCommand() {
        return new SequentialCommandGroup(
                AutoCommands.moveForwardMeters(mDrivetrain, 0.48, 1 / 3d), // Push cube
                AutoCommands.moveForwardMeters(mDrivetrain, -2, 1 / 3d) // Back up
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (AutoMap.kTranslatePidConstants != null) {
            builder.addDoubleProperty(
                    "Translate P",
                    () -> AutoMap.kTranslatePidConstants.kP,
                    (double newP) -> AutoMap.kTranslatePidConstants = new PIDConstants(newP,
                            AutoMap.kTranslatePidConstants.kI,
                            AutoMap.kTranslatePidConstants.kD));

            builder.addDoubleProperty(
                    "Translate I",
                    () -> AutoMap.kTranslatePidConstants.kI,
                    (double newI) -> AutoMap.kTranslatePidConstants = new PIDConstants(
                            AutoMap.kTranslatePidConstants.kP, newI,
                            AutoMap.kTranslatePidConstants.kD));

            builder.addDoubleProperty(
                    "Translate D",
                    () -> AutoMap.kTranslatePidConstants.kD,
                    (double newD) -> AutoMap.kTranslatePidConstants = new PIDConstants(
                            AutoMap.kTranslatePidConstants.kP,
                            AutoMap.kTranslatePidConstants.kI,
                            newD));
        }

        if (AutoMap.kRotatePidConstants != null) {
            builder.addDoubleProperty(
                    "Rotate P",
                    () -> AutoMap.kRotatePidConstants.kP,
                    (double newP) -> AutoMap.kRotatePidConstants = new PIDConstants(newP,
                            AutoMap.kRotatePidConstants.kI,
                            AutoMap.kRotatePidConstants.kD));

            builder.addDoubleProperty(
                    "Rotate I",
                    () -> AutoMap.kRotatePidConstants.kI,
                    (double newI) -> AutoMap.kRotatePidConstants = new PIDConstants(
                            AutoMap.kRotatePidConstants.kP, newI,
                            AutoMap.kRotatePidConstants.kD));

            builder.addDoubleProperty(
                    "Rotate D",
                    () -> AutoMap.kRotatePidConstants.kD,
                    (double newD) -> AutoMap.kRotatePidConstants = new PIDConstants(
                            AutoMap.kRotatePidConstants.kP,
                            AutoMap.kRotatePidConstants.kI,
                            newD));
        }

    }
}
