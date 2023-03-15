package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ForearmCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.DriveMap;

public class RobotContainer {
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

        mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve, mRearLeftSwerve, mRearRightSwerve);
        SmartDashboard.putData(mDrivetrain);

        mShoulder = new Shoulder();
        SmartDashboard.putData(mShoulder);

        mWrist = new Wrist();
        SmartDashboard.putData(mWrist);

        configureBindings();
    }

    private void configureBindings() {
        mDriveController = new CommandJoystick(0);
        mOperatorController = new CommandJoystick(1);
        SwerveModule[] modules = new SwerveModule[] {
                mFrontLeftSwerve,
                mFrontRightSwerve,
                mRearLeftSwerve,
                mRearRightSwerve
        };

        // Drive commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mDriveController, mDrivetrain, modules, true));
        mDriveController.button(3).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));

        // Shoulder commands
        mShoulder.setDefaultCommand(ShoulderCommands.getRunSimpleCommand(mShoulder, mOperatorController));

        // Forearm commands
        mForearm.setDefaultCommand(ForearmCommands.getRunSimpleCommand(mForearm, mOperatorController));

        // Wrist commands
        mOperatorController.button(1)
                .onTrue(WristCommands.runMotorSimpleCommand(mWrist))
                .onFalse(WristCommands.stopIntakeCommand(mWrist));

        mOperatorController.button(2)
                .onTrue(WristCommands.toggleActuatorCommand(mWrist));

        // mController.pov(0)
        // .onTrue(WristCommands.runIntakeCubeAndEjectConeCommand(mWrist, true))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));

        // mController.pov(180)
        // .onTrue(WristCommands.runIntakeConeAndEjectCubeCommand(mWrist,false))
        // .onFalse(WristCommands.stopIntakeCommand(mWrist));

    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
