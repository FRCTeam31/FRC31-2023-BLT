package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.WristMap;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {
    public static Command controlWithJoystick(Forearm forearm, DoubleSupplier operatorJoystick) {
        return Commands.run(() -> forearm.runSimple(operatorJoystick.getAsDouble()), forearm);
    }

    public static Command setForearmDistance(Forearm forearm, double setpoint) {
        return Commands.run(() -> forearm.setSetpoint(setpoint));
    }
}
