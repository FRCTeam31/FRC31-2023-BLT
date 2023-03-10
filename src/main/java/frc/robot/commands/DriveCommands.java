package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
    public static Command defaultDriveCommand(CommandJoystick controller, Drivetrain driveTrain,
            SwerveModule[] swerveModules, boolean fieldRelative) {
        return Commands.run(() -> {
            var strafeX = MathUtil.applyDeadband(controller.getRawAxis(0), 0.1);
            var forwardY = MathUtil.applyDeadband(controller.getRawAxis(1), 0.1);
            var rotation = controller.getRawAxis(2) - controller.getRawAxis(3);

            strafeX = MathUtil.clamp(strafeX, -DriveMap.slowDriveCoefficent, DriveMap.slowDriveCoefficent);
            forwardY = MathUtil.clamp(forwardY, -DriveMap.slowDriveCoefficent, DriveMap.slowDriveCoefficent);
            rotation = MathUtil.clamp(-rotation, -DriveMap.slowDriveCoefficent, DriveMap.slowDriveCoefficent);

            driveTrain.drive(-strafeX, forwardY, rotation, fieldRelative);
        }, driveTrain, swerveModules[0], swerveModules[1], swerveModules[2], swerveModules[3]);
    }

    public static Command resetGyroComamand(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.resetGyro(), driveTrain);
    }

    public static Command followTrajectoryWithEventCommand(Drivetrain drivetrain, PathPlannerTrajectory trajectory,
            boolean isFirstPath) {
        var thetaController = new PIDController(DriveMap.kAutonRotationKp, DriveMap.kAutonRotationKi,
                DriveMap.kAutonDriveXKd);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
                    }

                }),
                new PPSwerveControllerCommand(trajectory,
                        drivetrain::getPose,
                        drivetrain.mKinematics,
                        new PIDController(DriveMap.kAutonDriveXKp, DriveMap.kAutonDriveXKi, DriveMap.kAutonDriveXKd),
                        new PIDController(DriveMap.kAutonDriveYKp, DriveMap.kAutonDriveYKi, DriveMap.kAutonDriveYKd),
                        thetaController,
                        drivetrain::drive,
                        drivetrain));
    }
}
