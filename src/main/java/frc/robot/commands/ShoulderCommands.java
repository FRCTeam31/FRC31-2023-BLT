package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static SequentialCommandGroup setAngleCommand(Shoulder shoulder, double angle) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> shoulder.enable()),
                Commands.runOnce(() -> shoulder.setSetpoint(angle)));
    }

    public static Command controlWithJoystick(Shoulder shoulder, CommandJoystick operatorController) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> shoulder.disable()),
                Commands.run(() -> {
                    var joystickInput = operatorController.getRawAxis(ControlsMap.LEFT_STICK_Y);

                    shoulder.setShoulderSpeed(MathUtil.applyDeadband(joystickInput, ControlsMap.AXIS_DEADBAND));
                }, shoulder));
    }
}
