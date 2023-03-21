package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArduinoSidecar;

public class LightCommands {
    public static Command getSetFrontStripColor(ArduinoSidecar light, int red, int green, int blue) {
        return Commands.runOnce(() -> light.setFrontStripColor(red, green, blue), light);
    }
}
