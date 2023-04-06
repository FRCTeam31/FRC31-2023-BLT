package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class AutoScoringCommands {

    public Command ScoreMidCommand(Shoulder shoulder, Wrist wrist) {
        return new SequentialCommandGroup(
                ShoulderCommands.setHighGoal(shoulder),
                Commands.waitSeconds(0.75),
                WristCommands.EjectForTimeCommand(wrist, 1)

        );
    }
}
