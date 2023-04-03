package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
    public static Command defaultDriveCommand(Drivetrain drivetrain, DoubleSupplier ySupplier, DoubleSupplier xSupplier,
            DoubleSupplier rotationSupplier,
            SwerveModule[] swerveModules, boolean fieldRelative) {
        return Commands.run(() -> {
            var strafeX = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
            var forwardY = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
            var rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);

            drivetrain.drive(-strafeX, forwardY, rotation, fieldRelative);
        }, drivetrain);
    }

    public static Command resetGyroComamand(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.resetGyro(), driveTrain);
    }

    public static Command resetOdometry(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.resetOdometry(new Pose2d()), driveTrain);
    }

    public static Command toggleShifter(Drivetrain drive) {
        return Commands.runOnce(() -> drive.toggleShifter());
    }

    public static Command followTrajectoryWithEventsCommand(Drivetrain drivetrain,
            PathPlannerTrajectory trajectory,
            boolean isFirstPath, PIDController translationXController,
            PIDController translationYController,
            PIDController rotationController) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    if (isFirstPath) {
                        drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        drivetrain::getPose, // Pose supplier
                        translationXController, // X controller. Tune these values for your robot. Leaving them 0
                                                // will only use feedforwards.
                        translationYController, // Y controller (usually the same values as X controller)
                        rotationController, // Rotation controller. Tune these values for your robot. Leaving
                                            // them 0 will only use feedforwards.
                        drivetrain::drive,
                        drivetrain));
    }
}
