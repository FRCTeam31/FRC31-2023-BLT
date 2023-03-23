package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {
    // public static Command controlWithJoystick(Forearm forearm, CommandJoystick
    // driveJoystick) {
    // return Commands.runOnce(() -> {

    // }, forearm);
    // }

    public static Command controlWithJoystick(Forearm forearm, DoubleSupplier operatorJoystick) {
        return Commands.run(() -> forearm.runSimple(operatorJoystick.getAsDouble()), forearm);
    }

}
