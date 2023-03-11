package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.forArme;

public class forAmeCammands {
 


    public static Command getRunSimpleCommand(forArme forArme, CommandJoystick joystick){
        return new SequentialCommandGroup(
            getSetPIDEnabledCommand(forArme, false),
            Commands.run(() -> forArme.runforArme(joystick.getRawAxis(1)), forArme)
        );
    }

    public static Command getSetPIDEnabledCommand(forArme[] forArme, boolean enabled) {
        return Commands.runOnce(() -> forArme.setPIDEnabled(enabled), forArme);
    }

    public static Command getSetAngleCommand(forArme forArme, double angle) {
        return Commands.runOnce(() -> forArme.setSetpoint(angle));
    }
}
