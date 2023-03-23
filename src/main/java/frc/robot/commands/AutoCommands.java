package frc.robot.commands;

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

    public static Command moveForwardMeters(Drivetrain drivetrain, double yDistance, double speedMetersPerSecond) {
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

    public static Command moveSidewaysMeters(Drivetrain drivetrain, double xDistance, double speedMetersPerSecond) {
        return new SequentialCommandGroup(
                DriveCommands.resetOdometry(drivetrain),
                Commands.run(() -> {
                    var startPosition = drivetrain.getPose().getTranslation().getX(); // Grab our starting position
                    var destinationPosition = startPosition + xDistance; // Set our destination
                    var currentPosition = startPosition;

                    if (currentPosition < destinationPosition) {
                        while (currentPosition < destinationPosition) {
                            currentPosition = drivetrain.getPose().getTranslation().getX();
                            drivetrain.drive(new ChassisSpeeds(speedMetersPerSecond, 0, 0));
                        }
                    } else {
                        while (currentPosition > destinationPosition) {
                            currentPosition = drivetrain.getPose().getTranslation().getX();
                            drivetrain.drive(new ChassisSpeeds(speedMetersPerSecond, 0, 0));
                        }
                    }

                    drivetrain.stopMotors();
                }, drivetrain));
    }
}
