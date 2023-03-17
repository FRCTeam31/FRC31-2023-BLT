package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {
    public static Command getRunSimpleCommand(Forearm forearm, CommandJoystick joystick) {
        return Commands.run(() -> forearm.run(joystick.getRawAxis(1)), forearm);
    }

    public static Command getSetAngleCommand(Forearm forearm, double angle) {
        return Commands.runOnce(() -> forearm.setSetpoint(angle));
    }
}
