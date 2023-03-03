package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static Command getRunSimpleCommand(Shoulder shoulder, CommandJoystick joystick){
        return Commands.run(() -> shoulder.runShoulder(joystick.getRawAxis(5)), shoulder);
    }
}
