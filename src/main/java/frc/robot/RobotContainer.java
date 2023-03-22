package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.utilities.PathPlannerConverter;
import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.LightCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.AutoMap;
import frc.robot.config.ControlsMap;
import frc.robot.config.DriveMap;

public class RobotContainer implements Sendable {
    public CommandJoystick mDriverController;
    public CommandJoystick mOperatorController;
    public SwerveModule mFrontLeftSwerve;
    public SwerveModule mFrontRightSwerve;
    public SwerveModule mRearLeftSwerve;
    public SwerveModule mRearRightSwerve;
    public Drivetrain mDrivetrain;
    public Shoulder mShoulder;
    public Wrist mWrist;
    public Forearm mForearm;
    public ArduinoSidecar mSidecar;
    public UsbCamera mCamera;

    public RobotContainer() {
        mFrontLeftSwerve = new SwerveModule(
                DriveMap.kFrontLeftDrivingMotorId,
                DriveMap.kFrontLeftSteeringMotorId,
                DriveMap.kFrontLeftEncoderId,
                DriveMap.kFrontLeftEncoderOffset,
                DriveMap.kFrontLeftInverted);
        SmartDashboard.putData("Front Left Module", mFrontLeftSwerve);

        mFrontRightSwerve = new SwerveModule(
                DriveMap.kFrontRightDrivingMotorId,
                DriveMap.kFrontRightSteeringMotorId,
                DriveMap.kFrontRightEncoderId,
                DriveMap.kFrontRightEncoderOffset,
                DriveMap.kFrontRightInverted);
        SmartDashboard.putData("Front Right Module", mFrontRightSwerve);

        mRearLeftSwerve = new SwerveModule(
                DriveMap.kRearLeftDrivingMotorId,
                DriveMap.kRearLeftSteeringMotorId,
                DriveMap.kRearLeftEncoderId,
                DriveMap.kRearLeftEncoderOffset,
                DriveMap.kRearLeftInverted);
        SmartDashboard.putData("Rear Left Module", mRearLeftSwerve);

        mRearRightSwerve = new SwerveModule(
                DriveMap.kRearRightDrivingMotorId,
                DriveMap.kRearRightSteeringMotorId,
                DriveMap.kRearRightEncoderId,
                DriveMap.kRearRightEncoderOffset,
                DriveMap.kRearRightInverted);
        SmartDashboard.putData("Rear Right Module", mRearRightSwerve);

        mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve,
                mRearLeftSwerve, mRearRightSwerve);
        SmartDashboard.putData(mDrivetrain);

        mShoulder = new Shoulder();
        SmartDashboard.putData(mShoulder);

        mForearm = new Forearm();
        SmartDashboard.putData(mForearm);

        mWrist = new Wrist();
        SmartDashboard.putData(mWrist);

        SmartDashboard.putData(
                AutoMap.kTranslatePidConstantsName,
                PathPlannerConverter.fromPPPIDConstants(AutoMap.kTranslatePidConstants));

        SmartDashboard.putData(
                AutoMap.kRotatePidConstantsName,
                PathPlannerConverter.fromPPPIDConstants(AutoMap.kRotatePidConstants));
        SmartDashboard.putData(DriveMap.kDrivePidConstantsName,
                DriveMap.kDrivePidConstants);
        SmartDashboard.putData(DriveMap.kSteeringPidConstantsName,
                DriveMap.kSteeringPidConstants);
    }

    public void configureBindings() {
        mDriverController = new CommandJoystick(ControlsMap.DRIVER_PORT);
        mOperatorController = new CommandJoystick(ControlsMap.OPERATOR_PORT);
        SwerveModule[] modules = new SwerveModule[] {
                mFrontLeftSwerve,
                mFrontRightSwerve,
                mRearLeftSwerve,
                mRearRightSwerve
        };

        // Drive commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDriverController,
                mDrivetrain, modules, true));
        mDriverController.button(3).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));

        // mShoulder.setDefaultCommand(ShoulderCommands.controlWithJoystick(mShoulder,
        // mOperatorController));

        // Wrist bindings
        // mController.button(1)
        // .onTrue(WristCommands.runMotorSimpleCommand(mWrist))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));
        // mOperatorController.button(2)
        // .onTrue(WristCommands.toggleActuatorCommand(mWrist));

        // Shoulder commands
        mOperatorController.button(ControlsMap.Y)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kTopRowAngle));
        mOperatorController.button(ControlsMap.B)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kMiddleRowAngle));
        mOperatorController.button(ControlsMap.A)
                .onTrue(ShoulderCommands.setAngleCommand(mShoulder,
                        Shoulder.Map.kGroundLevelAngle));

        // Wrist commands
        mWrist.setDefaultCommand(WristCommands.runIntake(mWrist, mOperatorController));
        mOperatorController.pov(ControlsMap.up)
                .onTrue(WristCommands.setWristCommand(mWrist, true));

        mOperatorController.pov(ControlsMap.down)
                .onTrue(WristCommands.setWristCommand(mWrist, false));
    }

    // public SequentialCommandGroup getAutonomousCommand() {
    // PathPlannerTrajectory drive1Meter = PathPlanner.generatePath(
    // new PathConstraints(1, 0.1),
    // new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0),
    // Rotation2d.fromDegrees(0)),
    // new PathPoint(new Translation2d(0, 2), Rotation2d.fromDegrees(0),
    // Rotation2d.fromDegrees(0)));

    // ArrayList<PathPlannerTrajectory> pathGroup = new
    // ArrayList<PathPlannerTrajectory>();
    // pathGroup.add(drive1Meter);
    // HashMap<String, Command> eventMap = new HashMap<>();

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    // mDrivetrain::getPose,
    // mDrivetrain::resetOdometry,
    // mDrivetrain.mKinematics,
    // new PIDConstants(AutoMap.kTranslatePidConstants.kP,
    // AutoMap.kTranslatePidConstants.kI,
    // AutoMap.kTranslatePidConstants.kD),
    // new PIDConstants(AutoMap.kRotatePidConstants.kP,
    // AutoMap.kRotatePidConstants.kI,
    // AutoMap.kRotatePidConstants.kD),
    // mDrivetrain::drive,
    // eventMap);

    // return new SequentialCommandGroup(
    // Commands.runOnce(() ->
    // mDrivetrain.resetOdometry(drive1Meter.getInitialHolonomicPose())),
    // autoBuilder.fullAuto(pathGroup.get(0)));

    // }

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
