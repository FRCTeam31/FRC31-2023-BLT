package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.config.DriveMap;

public class RobotContainer {
    public CommandJoystick mController;
    public SwerveModule mFrontLeftSwerve;
    public SwerveModule mFrontRightSwerve;
    public SwerveModule mRearLeftSwerve;
    public SwerveModule mRearRightSwerve;
    public Drivetrain mDrivetrain;

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

        configureBindings();
    }

    private void configureBindings() {
        mController = new CommandJoystick(0);
        SwerveModule[] modules = new SwerveModule[]{
            mFrontLeftSwerve,
            mFrontRightSwerve,
            mRearLeftSwerve,
            mRearRightSwerve
        };

        // Default commands
        mDrivetrain.setDefaultCommand(DriveCommands.defaultDriveCommand(mController, mDrivetrain, modules, true));

        // Button bindings
        mController.button(3).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
