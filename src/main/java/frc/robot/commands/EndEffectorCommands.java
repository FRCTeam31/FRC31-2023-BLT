// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class EndEffectorCommands {

    // public static Command intakeConeCommand(Shoulder shoulder, Wrist wrist){

    // }

    public static Command intakeConeCommand(Shoulder shoulder, Wrist wrist, Forearm forearm) {
        return new SequentialCommandGroup(WristCommands.setWristCommand(wrist, false),
                ShoulderCommands.setAngleCommand(shoulder, Shoulder.Map.kGroundLevelAngle),
                ForearmCommands.setForearmDistance(forearm, Forearm.Map.pickUpGroundDistance));
    }

}
