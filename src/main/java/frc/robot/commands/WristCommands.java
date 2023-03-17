package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.WristMap;
import frc.robot.subsystems.Wrist;

public class WristCommands {
    public static Command runIntakeConeAndEjectCubeCommand(Wrist wrist, boolean out, CommandJoystick joystick) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(out), wrist),
                Commands.run(() -> wrist.runMotors(-joystick.getRawAxis(-5)), wrist)
        });
    }

    public static Command runIntakeCubeAndEjectConeCommand(Wrist wrist, boolean out, CommandJoystick joystick) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(out), wrist),
                Commands.run(() -> wrist.runMotors(joystick.getRawAxis(5)), wrist)
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
        return Commands.run(() -> wrist.runMotors(-joystick.getRawAxis(5)), wrist);
    }

}
