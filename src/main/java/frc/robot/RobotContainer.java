package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.commands.WristCommands;
import frc.robot.config.DriveMap;

public class RobotContainer {
        public CommandJoystick mDriverController;
        public CommandJoystick mOperatorController;
        public SwerveModule mFrontLeftSwerve;
        public SwerveModule mFrontRightSwerve;
        public SwerveModule mRearLeftSwerve;
        public SwerveModule mRearRightSwerve;
        public Drivetrain mDrivetrain;
        public Shoulder mShoulder;
        public Wrist mWrist;

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

                WristCommands.runIntakeConeAndEjectCubeCommand(mWrist, mDriverController, mOperatorController);
                WristCommands.runIntakeCubeAndEjectConeCommand(mWrist, mDriverController, mOperatorController);

                configureBindings();
        }

        private void configureBindings() {
                mDriverController = new CommandJoystick(0);
                mOperatorController = new CommandJoystick(1);
                SwerveModule[] modules = new SwerveModule[] {
                                mFrontLeftSwerve,
                                mFrontRightSwerve,
                                mRearLeftSwerve,
                                mRearRightSwerve

                };

                // Default commands
                mDrivetrain.setDefaultCommand(
                                DriveCommands.defaultDriveCommand(mDriverController, mDrivetrain, modules, true));
                mWrist.setDefaultCommand(WristCommands.controlWithJoystickCommand(mWrist, mDriverController));

                // Shoulder
                // mShoulder.setDefaultCommand(ShoulderCommands.getRunSimpleCommand(mShoulder,
                // mController));
                // mController.button(1).onTrue(Commands.runOnce(() -> {
                // mShoulder.enable();
                // mShoulder.setSetpoint(200);
                // }, mShoulder));

                // Button bindings
                mDriverController.button(3).onTrue(Commands.runOnce(() -> mDrivetrain.resetGyro()));

                // Wrist bindings
                // mController.button(1)
                // .onTrue(WristCommands.runMotorSimpleCommand(mWrist))
                // .onFalse(WristCommands.stopIntakeCommand(mWrist));

                // mController.button(2)
                // .onTrue(WristCommands.toggleActuatorCommand(mWrist));

                mOperatorController.pov(90)
                                .onTrue(WristCommands.setWristCommand(mWrist, true));

                mOperatorController.pov(180)
                                .onTrue(WristCommands.setWristCommand(mWrist, false));
        }

        public Command getAutonomousCommand() {
                return new InstantCommand();
        }
}
