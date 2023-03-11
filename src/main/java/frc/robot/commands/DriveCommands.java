package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
}
