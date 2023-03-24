package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.models.ShoulderLevels;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static Command setAngle(Shoulder shoulder, double angle) {
        return Commands.runOnce(() -> shoulder.setAngle(angle));
    }

    public static Command setAngle(Shoulder shoulder, ShoulderLevels level) {
        return Commands.runOnce(() -> shoulder.setAngle(level));
    }

    public static Command runWithJoystick(Shoulder shoulder, DoubleSupplier joystickSupplier) {
        return Commands.run(() -> shoulder.setSpeed(joystickSupplier.getAsDouble()), shoulder);
    }

    public static Command lockCurrentAngle(Shoulder shoulder) {
        return Commands.runOnce(() -> shoulder.setAngle(shoulder.getRotation().getDegrees()), shoulder);
    }

    // public static Command controlWithJoystick(Shoulder shoulder, CommandJoystick
    // operatorController) {
    // return new SequentialCommandGroup(
    // Commands.run(() -> {
    // var joystickInput = operatorController.getRawAxis(ControlsMap.LEFT_STICK_Y);

    // if (Math.abs(joystickInput) > ControlsMap.Driver)
    // shoulder.setShoulderSpeed(MathUtil.applyDeadband(joystickInput,
    // ControlsMap.AXIS_DEADBAND));
    // }, shoulder));
    // }

    public static Command togglePID(Shoulder shoulder) {
        return Commands.runOnce(() -> {
            if (shoulder.isEnabled()) {
                shoulder.disable();
            } else {
                shoulder.enable();
            }
        });
    }
}
