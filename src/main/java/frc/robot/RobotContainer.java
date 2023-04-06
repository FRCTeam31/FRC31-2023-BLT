package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.ArduinoSidecar.LEDMode;

import java.util.HashMap;

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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.SketchyAuto;
import frc.robot.commands.WristCommands;
import frc.robot.config.ControlsMap;
import frc.robot.models.IntakeDirection;

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
    public CommandJoystick mDriverController;
    public CommandJoystick mOperatorController;
    public Drivetrain mDrivetrain;
    public Shoulder mShoulder;
    public Compressor mCompressor;
    public SketchyAuto mSketchyAuto;
    public ArduinoSidecar mSidecar;
    public PowerDistribution mPdp;
    // public FrontCamera mFrontCamera;

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

        try {
            mSketchyAuto = new SketchyAuto();

        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize Autonomous subsystem",
                    false);
        }

        mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();
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
                () -> mOperatorController.getRawAxis(ControlsMap.RIGHT_TRIGGER) > Wrist.Map.kTriggerDeadband));

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
    }

    public Command getAutonomousCommand(double driveSpeed) {
        var eventMap = new HashMap<String, Command>();
        eventMap.putAll(WristCommands.getEvents(Wrist));

        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(mDrivetrain, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))),
                SketchyAuto.getAutonomousCommand(mShoulder, mWrist));
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
