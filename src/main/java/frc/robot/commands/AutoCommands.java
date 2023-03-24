package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutoCommands {
    public static Command moveForwardOneMeter(Drivetrain drivetrain) {
        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(drivetrain),
                Commands.run(() -> {
                    var position = drivetrain.getPose().getTranslation().getY();
                    while (position < 1) {
                        drivetrain.drive(new ChassisSpeeds(0, 1 / 3, 0));
                    }
                }, drivetrain));
    }

    public static Command translateYMeters(Drivetrain drivetrain, double yDistance, double speedMetersPerSecond) {
        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(drivetrain),
                Commands.run(() -> {
                    var startPosition = drivetrain.getPose().getTranslation().getY(); // Grab our starting position
                    var currentPosition = startPosition;

                    while (currentPosition < (startPosition + yDistance)) {
                        currentPosition = drivetrain.getPose().getTranslation().getY();
                        drivetrain.drive(new ChassisSpeeds(0, speedMetersPerSecond, 0));
                    }

                    drivetrain.stopMotors();
                }, drivetrain));
    }

    public static Command translateXMeters(Drivetrain drivetrain, double xDistanceMeters, double timeoutSeconds) {
        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(drivetrain),
                Commands.run(() -> {
                    var speedMetersPerSecond = xDistanceMeters / timeoutSeconds;

                    if (xDistanceMeters > 0) {
                        while (drivetrain.getPose().getTranslation().getX() < xDistanceMeters) {
                            drivetrain.drive(new ChassisSpeeds(speedMetersPerSecond, 0, 0));
                        }

                        System.out.println("Finished translating " + xDistanceMeters + "m in +X");
                    } else {
                        while (drivetrain.getPose().getTranslation().getX() > xDistanceMeters) {
                            drivetrain.drive(new ChassisSpeeds(-speedMetersPerSecond, 0, 0));
                        }

                        System.out.println("Finished translating " + xDistanceMeters + "m in -X");
                    }

                    drivetrain.stopMotors();
                }, drivetrain).withTimeout(timeoutSeconds));
    }

    public static Command rotateDegrees(Drivetrain drivetrain, double desiredHeadingDegrees, double timeoutSeconds) {
        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(drivetrain),
                Commands.run(() -> {
                    var speedDegreesPerSecond = desiredHeadingDegrees / timeoutSeconds;
                    var speedRadiansPerSecond = Rotation2d.fromDegrees(speedDegreesPerSecond).getRadians();

                    while (drivetrain.getPose().getRotation().getDegrees() != desiredHeadingDegrees) {
                        drivetrain.drive(new ChassisSpeeds(0, 0, speedRadiansPerSecond));
                    }

                }, drivetrain).withTimeout(timeoutSeconds));
    }
}