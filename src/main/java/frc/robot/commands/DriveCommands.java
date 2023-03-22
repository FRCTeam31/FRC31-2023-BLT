package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
    public static Command defaultDriveCommand(CommandJoystick controller, Drivetrain driveTrain,
            SwerveModule[] swerveModules, boolean fieldRelative) {
        return Commands.run(() -> {
            var strafeX = MathUtil.applyDeadband(controller.getRawAxis(ControlsMap.RIGHT_STICK_X), 0.1);
            var forwardY = MathUtil.applyDeadband(controller.getRawAxis(ControlsMap.RIGHT_STICK_Y), 0.1);
            var rotation = MathUtil.applyDeadband(controller.getRawAxis(ControlsMap.LEFT_STICK_X), 0.1);

            // strafeX = MathUtil.clamp(strafeX, -DriveMap.slowDriveCoefficent,
            // DriveMap.slowDriveCoefficent);
            // forwardY = MathUtil.clamp(forwardY, -DriveMap.slowDriveCoefficent,
            // DriveMap.slowDriveCoefficent);
            // rotation = MathUtil.clamp(-rotation, -DriveMap.slowDriveCoefficent,
            // DriveMap.slowDriveCoefficent);

            // var gearCoefficient = driveTrain.getShiftedSpeedCoefficient();
            // strafeX = MathUtil.clamp(strafeX, -gearCoefficient, gearCoefficient);
            // forwardY = MathUtil.clamp(forwardY, -gearCoefficient, gearCoefficient);
            // rotation = MathUtil.clamp(rotation, -gearCoefficient, gearCoefficient);

            driveTrain.drive(-strafeX, forwardY, -rotation, fieldRelative);
        }, driveTrain, swerveModules[0], swerveModules[1], swerveModules[2], swerveModules[3]);
    }

    public static Command resetGyroComamand(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.resetGyro(), driveTrain);
    }

    public static Command toggleShifter(Drivetrain drive) {
        return Commands.runOnce(() -> drive.toggleShifter());
    }

    // public static Command followTrajectoryWithEventsCommand(Drivetrain
    // drivetrain, PathPlannerTrajectory trajectory, boolean isFirstPath) {
    // return new SequentialCommandGroup(
    // new InstantCommand(() -> {
    // // Reset odometry for the first path you run during auto
    // if (isFirstPath) {
    // drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
    // }
    // }),
    // new PPSwerveControllerCommand(
    // trajectory,
    // drivetrain::getPose, // Pose supplier
    // new PIDController(0, 0, 0), // X controller. Tune these values for your
    // robot. Leaving them 0 will only use feedforwards.
    // new PIDController(0, 0, 0), // Y controller (usually the same values as X
    // controller)
    // new PIDController(0, 0, 0), // Rotation controller. Tune these values for
    // your robot. Leaving
    // // them 0 will only use feedforwards.
    // drivetrain::drive,
    // drivetrain));
    // public static Command followTrajectoryWithEventsCommand(Drivetrain
    // drivetrain, PathPlannerTrajectory trajectory, boolean isFirstPath) {
    // return new SequentialCommandGroup(
    // new InstantCommand(() -> {
    // // Reset odometry for the first path you run during auto
    // if (isFirstPath) {
    // drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
    // }
    // }),
    // new PPSwerveControllerCommand(
    // trajectory,
    // drivetrain::getPose, // Pose supplier
    // new PIDController(0, 0, 0), // X controller. Tune these values for your
    // robot. Leaving them 0 will only use feedforwards.
    // new PIDController(0, 0, 0), // Y controller (usually the same values as X
    // controller)
    // new PIDController(0, 0, 0), // Rotation controller. Tune these values for
    // your robot. Leaving
    // // them 0 will only use feedforwards.
    // drivetrain::drive,
    // drivetrain));
    // }
}
