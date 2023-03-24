// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.WristMap;
import frc.robot.models.ShoulderLevels;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class EndEffectorCommands {

    // public static Command intakeConeCommand(Shoulder shoulder, Wrist wrist){

    // }

    // public static Command IntakeGamePiece(Shoulder shoulder, Wrist wrist, Forearm
    // forearm, int intakeDirection) {
    // return new SequentialCommandGroup(WristCommands.setWristCommand(wrist,
    // false),
    // ShoulderCommands.setAngleCommand(shoulder, Shoulder.Map.kGroundAngle),
    // ForearmCommands.setForearmDistance(forearm,
    // Forearm.Map.pickUpGroundDistance),
    // WristCommands.runWristForTimeCommand(wrist, 3,
    // intakeDirection),
    // WristCommands.stopIntakeCommand(wrist));
    // }

    // public static Command ScoreCubeCommand(Shoulder shoulder, Wrist wrist,
    // Forearm foearm, ShoulderLevels level) {
    // double shoulderAngle = shoulder.getMeasurement();
    // if (level == ShoulderLevels.GROUND) {
    // shoulderAngle = Shoulder.Map.kMiddleRowAngle;
    // } else if (level == ShoulderLevels.SCORE_HIGH) {
    // shoulderAngle = Shoulder.Map.kTopRowAngle;
    // }
    // return new SequentialCommandGroup(
    // ShoulderCommands.setAngleCommand(shoulder, shoulderAngle),
    // WristCommands.setWristCommand(wrist, true),
    // WristCommands.runWristForTimeCommand(wrist, 3, WristMap.kEjectCubeSpeed),
    // WristCommands.stopIntakeCommand(wrist));
    // }

    // public static Command ScoreConeCommand(Shoulder shoulder, Wrist wrist,
    // Forearm forearm, ShoulderLevels level) {
    // double shoulderAngle = shoulder.getMeasurement();
    // if (level == ShoulderLevels.GROUND) {
    // shoulderAngle = Shoulder.Map.kMiddleRowAngle;
    // } else if (level == ShoulderLevels.SCORE_HIGH) {
    // shoulderAngle = Shoulder.Map.kTopRowAngle;
    // }
    // return new SequentialCommandGroup(ShoulderCommands.setAngleCommand(shoulder,
    // shoulderAngle),
    // WristCommands.setWristCommand(wrist, false),
    // WristCommands.runWristForTimeCommand(wrist, 3, WristMap.kEjectConeSpeed),
    // WristCommands.stopIntakeCommand(wrist));
    // }
}
