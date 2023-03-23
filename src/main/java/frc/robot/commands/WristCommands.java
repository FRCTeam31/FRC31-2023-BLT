package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.config.WristMap;
import frc.robot.subsystems.Wrist;

public class WristCommands {
    public static Command runIntake(Wrist wrist, CommandJoystick operatorController) {
        return Commands.run(() -> {
            if (operatorController.getRawAxis(ControlsMap.LEFT_TRIGGER) > WristMap.triggerDeadBand) {
                wrist.runMotors(WristMap.kEjectConeSpeed);
                return;
            } else if (operatorController.getRawAxis(ControlsMap.RIGHT_TRIGGER) > WristMap.triggerDeadBand) {
                wrist.runMotors(WristMap.kIntakeConeSpeed);
                return;
            } else {
                wrist.runMotors(0);
            }

        }, wrist);
    }

    public static Command setWristCommand(Wrist wrist, boolean out) {
        return Commands.runOnce(() -> {
            wrist.setWrist(out);
        });
    }

    public static Command runIntakeFallenConeCommand(Wrist wrist, boolean out) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(false), wrist),
                Commands.run(() -> wrist.runMotors(-WristMap.kIntakeConeSpeed), wrist)
        });
    }

    public static Command stopIntakeCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.stopIntake(), wrist);
    }

    // Test commands
    public static Command runMotorSimpleCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.runMotors(1), wrist);
    }

    public static Command toggleActuatorCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.setWrist(!wrist.getWristOut()), wrist);
    }

    public static Command controlWithJoystickCommand(Wrist wrist, CommandJoystick joystick) {
        return Commands.run(() -> wrist.runMotors(-joystick.getRawAxis(ControlsMap.RIGHT_STICK_Y)), wrist);
    }

    public static Command runWristForTimeCommand(Wrist wrist, double seconds, double speed) {
        return Commands.run(() -> {
            wrist.runMotors(speed);
        }).withTimeout(seconds);
    }

}
