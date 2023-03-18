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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.LightCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.config.AutoMap;
import frc.robot.config.DriveMap;

public class RobotContainer implements Sendable {
    public CommandJoystick mDriveController;
    public CommandJoystick mOperatorController;
    public SwerveModule mFrontLeftSwerve;
    public SwerveModule mFrontRightSwerve;
    public SwerveModule mRearLeftSwerve;
    public SwerveModule mRearRightSwerve;
    public Drivetrain mDrivetrain;
    public Shoulder mShoulder;
    public Wrist mWrist;
    public Forearm mForearm;

    public RobotContainer() {
        // mFrontLeftSwerve = new SwerveModule(
        // DriveMap.kFrontLeftDrivingMotorId,
        // DriveMap.kFrontLeftSteeringMotorId,
        // DriveMap.kFrontLeftEncoderId,
        // DriveMap.kFrontLeftEncoderOffset,
        // DriveMap.kFrontLeftInverted);
        // SmartDashboard.putData("Front Left Module", mFrontLeftSwerve);

        // mFrontRightSwerve = new SwerveModule(
        // DriveMap.kFrontRightDrivingMotorId,
        // DriveMap.kFrontRightSteeringMotorId,
        // DriveMap.kFrontRightEncoderId,
        // DriveMap.kFrontRightEncoderOffset,
        // DriveMap.kFrontRightInverted);
        // SmartDashboard.putData("Front Right Module", mFrontRightSwerve);

        // mRearLeftSwerve = new SwerveModule(
        // DriveMap.kRearLeftDrivingMotorId,
        // DriveMap.kRearLeftSteeringMotorId,
        // DriveMap.kRearLeftEncoderId,
        // DriveMap.kRearLeftEncoderOffset,
        // DriveMap.kRearLeftInverted);
        // SmartDashboard.putData("Rear Left Module", mRearLeftSwerve);

        // mRearRightSwerve = new SwerveModule(
        // DriveMap.kRearRightDrivingMotorId,
        // DriveMap.kRearRightSteeringMotorId,
        // DriveMap.kRearRightEncoderId,
        // DriveMap.kRearRightEncoderOffset,
        // DriveMap.kRearRightInverted);
        // SmartDashboard.putData("Rear Right Module", mRearRightSwerve);

        // mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve,
        // mRearLeftSwerve, mRearRightSwerve);
        // SmartDashboard.putData(mDrivetrain);

        mShoulder = new Shoulder();
        SmartDashboard.putData(mShoulder);

        mForearm = new Forearm();
        SmartDashboard.putData(mForearm);

        // mWrist = new Wrist();
        // SmartDashboard.putData(mWrist);

        // SmartDashboard.putData(
        // AutoMap.kTranslatePidConstantsName,
        // PathPlannerConverter.fromPPPIDConstants(AutoMap.kTranslatePidConstants));

        // SmartDashboard.putData(
        // AutoMap.kRotatePidConstantsName,
        // PathPlannerConverter.fromPPPIDConstants(AutoMap.kRotatePidConstants));
        // SmartDashboard.putData(DriveMap.kDrivePidConstantsName,
        // DriveMap.kDrivePidConstants);
        // SmartDashboard.putData(DriveMap.kSteeringPidConstantsName,
        // DriveMap.kSteeringPidConstants);

        // Light lights = new Light();

        configureBindings();
        // LightCommands.getSetFrontStripColor(lights, 0, 255, 0);
    }

    private void configureBindings() {
        mDriveController = new CommandJoystick(0);
        mOperatorController = new CommandJoystick(1);
        // SwerveModule[] modules = new SwerveModule[] {
        // mFrontLeftSwerve,
        // mFrontRightSwerve,
        // mRearLeftSwerve,
        // mRearRightSwerve
        // };

        // Drive commands
        // mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDriveController,
        // mDrivetrain, modules, true));
        // mDriveController.button(3).onTrue(Commands.runOnce(() ->
        // mDrivetrain.resetGyro()));

        // Shoulder commands
        mOperatorController.pov(0).onTrue(Commands.runOnce(() -> mShoulder.setSetpoint(mShoulder.getSetpoint() + 10)));
        mOperatorController.pov(180)
                .onTrue(Commands.runOnce(() -> mShoulder.setSetpoint(mShoulder.getSetpoint() - 10)));
        // mOperatorController.button(7).onTrue(Commands.runOnce(() -> {
        // if (mShoulder.isEnabled())
        // mShoulder.disable();
        // else
        // mShoulder.enable();
        // }));

        // Forearm commands
        // mForearm.setDefaultCommand(ForearmCommands.getRunSimpleCommand(mForearm,
        // mOperatorController));
        // mOperatorController.button(8).onTrue(Commands.runOnce(() -> {
        // if (mForearm.isEnabled())
        // mForearm.disable();
        // else
        // mForearm.enable();
        // }));

        // Wrist commands
        // mOperatorController.button(1)
        // .onTrue(WristCommands.runMotorSimpleCommand(mWrist))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));

        // mOperatorController.button(2)
        // .onTrue(WristCommands.toggleActuatorCommand(mWrist));

        // mController.pov(0)
        // .onTrue(WristCommands.runIntakeCubeAndEjectConeCommand(mWrist, true))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));

        // mController.pov(180)
        // .onTrue(WristCommands.runIntakeConeAndEjectCubeCommand(mWrist,false))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));

        // Button bindings
        // mDriveController.button(3).onTrue(Commands.runOnce(() ->
        // mDrivetrain.resetGyro()));

        // mDriveController.button(1).onTrue(getAutonomousCommand());
    }

    public SequentialCommandGroup getAutonomousCommand() {
        PathPlannerTrajectory drive1Meter = PathPlanner.generatePath(
                new PathConstraints(1, 0.1),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0, 2), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)));

        ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<PathPlannerTrajectory>();
        pathGroup.add(drive1Meter);
        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                mDrivetrain::getPose,
                mDrivetrain::resetOdometry,
                mDrivetrain.mKinematics,
                new PIDConstants(AutoMap.kTranslatePidConstants.kP,
                        AutoMap.kTranslatePidConstants.kI,
                        AutoMap.kTranslatePidConstants.kD),
                new PIDConstants(AutoMap.kRotatePidConstants.kP,
                        AutoMap.kRotatePidConstants.kI,
                        AutoMap.kRotatePidConstants.kD),
                mDrivetrain::drive,
                eventMap);

        return new SequentialCommandGroup(
                Commands.runOnce(() -> mDrivetrain.resetOdometry(drive1Meter.getInitialHolonomicPose())),
                autoBuilder.fullAuto(pathGroup.get(0)));

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // if (AutoMap.kTranslatePidConstants != null) {
        // builder.addDoubleProperty(
        // "Translate P",
        // () -> AutoMap.kTranslatePidConstants.kP,
        // (double newP) -> AutoMap.kTranslatePidConstants = new PIDConstants(newP,
        // AutoMap.kTranslatePidConstants.kI,
        // AutoMap.kTranslatePidConstants.kD));

        // builder.addDoubleProperty(
        // "Translate I",
        // () -> AutoMap.kTranslatePidConstants.kI,
        // (double newI) -> AutoMap.kTranslatePidConstants = new PIDConstants(
        // AutoMap.kTranslatePidConstants.kP, newI,
        // AutoMap.kTranslatePidConstants.kD));

        // builder.addDoubleProperty(
        // "Translate D",
        // () -> AutoMap.kTranslatePidConstants.kD,
        // (double newD) -> AutoMap.kTranslatePidConstants = new PIDConstants(
        // AutoMap.kTranslatePidConstants.kP,
        // AutoMap.kTranslatePidConstants.kI,
        // newD));
        // }

        // if (AutoMap.kRotatePidConstants != null) {
        // builder.addDoubleProperty(
        // "Rotate P",
        // () -> AutoMap.kRotatePidConstants.kP,
        // (double newP) -> AutoMap.kRotatePidConstants = new PIDConstants(newP,
        // AutoMap.kRotatePidConstants.kI,
        // AutoMap.kRotatePidConstants.kD));

        // builder.addDoubleProperty(
        // "Rotate I",
        // () -> AutoMap.kRotatePidConstants.kI,
        // (double newI) -> AutoMap.kRotatePidConstants = new PIDConstants(
        // AutoMap.kRotatePidConstants.kP, newI,
        // AutoMap.kRotatePidConstants.kD));

        // builder.addDoubleProperty(
        // "Rotate D",
        // () -> AutoMap.kRotatePidConstants.kD,
        // (double newD) -> AutoMap.kRotatePidConstants = new PIDConstants(
        // AutoMap.kRotatePidConstants.kP,
        // AutoMap.kRotatePidConstants.kI,
        // newD));
        // }

    }
}
