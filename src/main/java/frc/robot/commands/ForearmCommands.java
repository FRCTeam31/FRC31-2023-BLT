package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {
    public static Command extendForearm(Forearm forearm) {
        return Commands.run(() -> {
            forearm.extendForearmSolenoid(true);
        }, forearm);
    }

    public static Command retractForearm(Forearm forearm) {
        return Commands.run(() -> {
            forearm.extendForearmSolenoid(false);
        }, forearm);
    }

}
