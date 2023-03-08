package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist;

public class WristCommands {
    public static Command runWristCommand(Wrist wrist, int direction){
        return Commands.run(() -> wrist.pushAndPull(direction), wrist) ;

    }

    public static Command stopWristCommand(Wrist wrist){
        return Commands.run(() -> wrist.stopForearm());
    }

    
    
}
