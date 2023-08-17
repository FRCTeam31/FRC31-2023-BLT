package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ArduinoSidecar.LEDMode;
import frc.robot.subsystems.Wrist.WristMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.config.ControlsMap;

public class RobotContainer implements Sendable {
    public Drivetrain Drivetrain;
    public Shoulder Shoulder;
    public Compressor Compressor;
    public Wrist Wrist;
    public Forearm Forearm;
    public ArduinoSidecar Sidecar;
    public PowerDistribution Pdp;
    public CommandJoystick DriverController;
    public CommandJoystick OperatorController;

    public RobotContainer() {
        try {
            Pdp = new PowerDistribution();
            Pdp.resetTotalEnergy();
            Pdp.clearStickyFaults();

            SmartDashboard.putData(Pdp);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize PowerDistribution subsystem",
                    true);
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
            Shoulder = new Shoulder();
            Shoulder.register();

            SmartDashboard.putData(Shoulder);
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

        Compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        Compressor.enableDigital();
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
        // DriverController.button(ControlsMap.B).onTrue(DriveCommands.enableSnapToGyroControl(Drivetrain));
        DriverController.button(ControlsMap.A).onTrue(DriveCommands.toggleSnapToAngleCommand(Drivetrain));

        // Shoulder commands
        OperatorController.button(ControlsMap.Y).onTrue(ShoulderCommands.setHighGoal(Shoulder));
        OperatorController.button(ControlsMap.B).onTrue(ShoulderCommands.setMiddleGoal(Shoulder));
        OperatorController.button(ControlsMap.A).onTrue(ShoulderCommands.setLowGoal(Shoulder));
        OperatorController.button(ControlsMap.X).onTrue(ShoulderCommands.setGround(Shoulder));

        // Forearm commands
        OperatorController.button(ControlsMap.RB).onTrue(ForearmCommands.extendForearm(Forearm));
        OperatorController.button(ControlsMap.LB).onTrue(ForearmCommands.retractForearm(Forearm));

        // Snap To Commands

        DriverController.pov(ControlsMap.up).onTrue(DriveCommands.driveWithSnapToAngleCommand(Drivetrain,
                0));
        DriverController.pov(ControlsMap.right).onTrue(DriveCommands.driveWithSnapToAngleCommand(Drivetrain,
                90));

        DriverController.pov(ControlsMap.down).onTrue(DriveCommands.driveWithSnapToAngleCommand(Drivetrain,
                180));

        DriverController.pov(ControlsMap.left).onTrue(DriveCommands.driveWithSnapToAngleCommand(Drivetrain,
                270));

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

    public LEDMode getLEDMode() {
        return Sidecar.getMode();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
