package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class SketchyAuto {

    public SketchyAuto() {

    }

    public static Command getAutonomousCommand(Shoulder shoulder, Wrist wrist) {
        return new SequentialCommandGroup(
                ShoulderCommands.setHighGoal(shoulder),
                WristCommands.EjectForTimeCommand(wrist)

        );
    }
}
