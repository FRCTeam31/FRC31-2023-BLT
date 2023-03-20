package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.config.ShoulderMap;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static Command getRunSimpleCommand(Shoulder shoulder, CommandJoystick joystick) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> shoulder.disable()),
                Commands.run(() -> shoulder.runShoulder(joystick.getRawAxis(ControlsMap.RIGHT_STICK_Y)), shoulder));
    }

    public static Command setAngleCommand(Shoulder shoulder, double angle) {
        return Commands.runOnce(() -> shoulder.setSetpoint(angle));
    }

    public static Command controlWithJoystick(Shoulder shoulder, CommandJoystick driveController) {
        return Commands.runOnce(() -> {

            var setpoint = 200 + (driveController.getRawAxis(ControlsMap.LEFT_STICK_Y) * 20);

            shoulder.setShoulderAngle(setpoint);

        });
    }

    public static Command controlWithBothJoysticks(Shoulder shoulder, CommandJoystick driveController,
            CommandJoystick operatorController) {
        return Commands.runOnce(() -> {
            if (driveController.getRawAxis(ControlsMap.LEFT_STICK_Y) > ShoulderMap.shoulderDeadband) {
                var setpoint = 200 + (driveController.getRawAxis(ControlsMap.LEFT_STICK_Y) * 20);

                shoulder.setShoulderAngle(setpoint);

            } else if (operatorController.getRawAxis(ControlsMap.LEFT_STICK_Y) > ShoulderMap.shoulderDeadband) {

                var setpoint = 200 + (driveController.getRawAxis(ControlsMap.LEFT_STICK_Y) * 20);

                shoulder.setShoulderAngle(setpoint);

            }

        });
    }
}
