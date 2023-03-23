package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArduinoSidecar;
import frc.robot.subsystems.ArduinoSidecar.LEDStrips;
import frc.robot.subsystems.ArduinoSidecar.StripModes;

public class LightCommands {
    public static Command getSetFrontStripColor(ArduinoSidecar light, int red, int green, int blue) {
        return Commands.runOnce(() -> light.setLEDStripMode(LEDStrips.FRONT, StripModes.SOLID_GREEN), light);
    }
}
