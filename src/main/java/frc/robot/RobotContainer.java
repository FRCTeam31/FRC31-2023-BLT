package frc.robot;

import frc.robot.subsystems.*;

import java.util.HashMap;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.Autonomous;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EndEffectorCommands;
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
    public Compressor mCompressor;
    // public Autonomous mAuto;

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
            mPdp = new PowerDistribution();
        } catch (Exception e) {
            DriverStation.reportError("Failed to initalize Powerboard", true);
            return;
        }

        // try {
        // mAuto = new Autonomous();

        // } catch (Exception e) {
        // DriverStation.reportError("Failed to initialize Autonomous subsystem",
        // false);
        // }

        // mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();

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
                () -> mOperatorController.getRawAxis(ControlsMap.LEFT_TRIGGER) > Wrist.Map.kTriggerDeadband,
                () -> false));

        // Drive commands
        mDriverController.button(ControlsMap.X).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro(), mDrivetrain));
        mDriverController.button(ControlsMap.Y).onTrue(DriveCommands.toggleShifter(mDrivetrain));

        // Shoulder commands
        mOperatorController.button(ControlsMap.Y).onTrue(ShoulderCommands.setHighGoal(mShoulder));
        mOperatorController.button(ControlsMap.B).onTrue(ShoulderCommands.setMiddleGoal(mShoulder));
        mOperatorController.button(ControlsMap.A).onTrue(ShoulderCommands.setLowGoal(mShoulder));
        mOperatorController.button(ControlsMap.X).onTrue(ShoulderCommands.setGround(mShoulder));

        // Forearm commands

        mOperatorController.button(ControlsMap.RB).onTrue(ForearmCommands.extendForearm(mForearm));
        mOperatorController.button(ControlsMap.LB).onTrue(ForearmCommands.retractForearm(mForearm));

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

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub

    }
}
