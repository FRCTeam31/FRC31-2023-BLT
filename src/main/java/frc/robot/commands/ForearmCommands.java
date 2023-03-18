package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {
    // public static Command getRunSimpleCommand(Forearm forearm, CommandJoystick
    // driverJoystick,
    // CommandJoystick operatorJoystick) {
    // return new SequentialCommandGroup(
    // Commands.runOnce(() -> forearm.disable()),
    // Commands.run(() -> forearm.run(driverJoystick.getRawAxis(1)), forearm));
    // }

    public static Command getSetAngleCommand(Forearm forearm, double angle) {
        return Commands.runOnce(() -> forearm.setSetpoint(angle));
    }

    public static Command controlWithJoystick(Forearm forearm, CommandJoystick driveJoystick) {
        return Commands.runOnce(() -> {

            var setpoint = 200 + (driveJoystick.getRawAxis(1) * 20);

            forearm.setSetpoint(setpoint);
        });
    }
}
