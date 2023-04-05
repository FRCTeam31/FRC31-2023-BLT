package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ArduinoSidecar.LEDMode;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.ControlsMap;

public class RobotContainer implements Sendable {
    public CommandJoystick DriverController;
    public CommandJoystick OperatorController;
    public Drivetrain Drivetrain;
    public Shoulder Shoulder;
    public Compressor Compressor;
    public Wrist Wrist;
    public Forearm Forearm;
    public ArduinoSidecar Sidecar;
    public PowerDistribution Pdp;

    public RobotContainer() {
        try {
            Pdp = new PowerDistribution();
            SmartDashboard.putData(Pdp);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize PowerDistribution subsystem",
                    true);
            return;
        }

        try {
            Drivetrain = new Drivetrain();
            SmartDashboard.putData(Drivetrain);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Drivetrain subsystem", true);
            return;
        }

        try {
            Shoulder = new Shoulder();
            SmartDashboard.putData(Shoulder);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Shoulder subsystem", true);
            return;
        }

        try {
            Forearm = new Forearm();
            SmartDashboard.putData(Forearm);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Forearm subsystem", true);
            return;
        }

        try {
            Wrist = new Wrist();
            SmartDashboard.putData(Wrist);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
            return;
        }

        try {
            Sidecar = new ArduinoSidecar();
            SmartDashboard.putData(Sidecar);
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Wrist subsystem", true);
            return;
        }

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
                () -> OperatorController.getRawAxis(ControlsMap.LEFT_TRIGGER) > Wrist.Map.kTriggerDeadband,
                () -> false));

        // Drive commands
        DriverController.button(ControlsMap.X).onTrue(Commands.runOnce(() -> Drivetrain.resetGyro(), Drivetrain));
        DriverController.button(ControlsMap.Y).onTrue(DriveCommands.toggleShifter(Drivetrain));

        // Shoulder commands
        OperatorController.button(ControlsMap.Y).onTrue(ShoulderCommands.setHighGoal(Shoulder));
        OperatorController.button(ControlsMap.B).onTrue(ShoulderCommands.setMiddleGoal(Shoulder));
        OperatorController.button(ControlsMap.A).onTrue(ShoulderCommands.setLowGoal(Shoulder));
        OperatorController.button(ControlsMap.X).onTrue(ShoulderCommands.setGround(Shoulder));

        // Forearm commands

        OperatorController.button(ControlsMap.RB).onTrue(ForearmCommands.extendForearm(Forearm));
        OperatorController.button(ControlsMap.LB).onTrue(ForearmCommands.retractForearm(Forearm));

        // End efector Commands

        // mOperatorController.button(ControlsMap.LOGO_RIGHT).onTrue(EndEffectorCommands.setGround(mShoulder,
        // mForearm));

        // mOperatorController.button(ControlsMap.LB)
        // .whileTrue(EndEffectorCommands.raiseEffectorManually(mShoulder, // While LB
        // is held, control the arm
        // // speed with the left stick Y axis
        // () -> mForearm.getMinSoftLimitReached(),
        // () -> mOperatorController.getRawAxis(ControlsMap.LEFT_STICK_Y)))
        // .onFalse(ShoulderCommands.lockCurrentAngle(mShoulder)); // When LB is
        // released, set the shoulder

        // Auto testing commands, only enabled when we're not on the field
        // if (!DriverStation.isFMSAttached()) {
        // var autoDriveSpeed = 1 / 4d;
        // mDriverController.button(ControlsMap.LB)
        // .onTrue(mAuto.getAutonomousCommand(mDrivetrain));
        // }
        // }

        // public SequentialCommandGroup getAutonomousCommand(double driveSpeed) {
        // var eventMap = new HashMap<String, Command>();
        // eventMap.putAll(WristCommands.getEvents(mWrist));

    }

    public Command getAutonomousCommand() {
        return null;
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
