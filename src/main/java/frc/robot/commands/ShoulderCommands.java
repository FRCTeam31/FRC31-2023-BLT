package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static Command setAngleCommand(Shoulder shoulder, double angle) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> shoulder.enable()),
                Commands.runOnce(() -> shoulder.setSetpoint(angle)));
    }

    public static Command disablePidAndRunManually(Shoulder shoulder, DoubleSupplier joystickSupplier) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> shoulder.disable()),
                runWithJoystick(shoulder, joystickSupplier));
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
}
