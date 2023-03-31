package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.WristMap;
import frc.robot.models.IntakeDirection;
import frc.robot.models.ShoulderLevels;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

/**
 * Commands which combine the shoulder, forearem, and wrist to facilitate and
 * perform effector actions
 */
public class EndEffectorCommands {
    /***
     * Scores a Cube on the Bottom row of the grid.
     * 
     * @param shoulder
     * @param forearm
     * @param wrist
     * @return
     */
    public static Command scoreCubeLowGoal(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        return new SequentialCommandGroup(
                ShoulderCommands.setLowGoal(shoulder),
                ForearmCommands.extendForearm(forearm),
                WristCommands.ejectCubeCommand(wrist).withTimeout(Wrist.Map.kEjectCubeTime),
                WristCommands.stopMotorsCommand(wrist),
                ForearmCommands.retractForearm(forearm));
    }

    /***
     * Scores a Cube on the middle row of the grid.
     * 
     * @param shoulder
     * @param forearm
     * @param wrist
     * @return
     */
    public static Command scoreCubeMidGoal(Shoulder shoulder, Forearm forearm, Wrist wrist) {

        return new SequentialCommandGroup(
                ShoulderCommands.setMiddleGoal(shoulder),
                ForearmCommands.extendForearm(forearm),
                WristCommands.ejectCubeCommand(wrist).withTimeout(Wrist.Map.kEjectCubeTime),
                WristCommands.stopMotorsCommand(wrist),
                ForearmCommands.retractForearm(forearm));

    }

    /***
     * Scores a Cube on the highest row of the grid.
     * 
     * @param shoulder
     * @param forearm
     * @param wrist
     * @return
     */
    public static Command scoreCubeHighGoal(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        return new SequentialCommandGroup(
                ShoulderCommands.setHighGoal(shoulder),
                ForearmCommands.extendForearm(forearm),
                WristCommands.shootCubeCommand(wrist).withTimeout(Wrist.Map.kEjectCubeTime),
                WristCommands.stopMotorsCommand(wrist),
                ForearmCommands.retractForearm(forearm));
    }

    /***
     * Picks up a Cube.
     * 
     * @param shoulder
     * @param forearm
     * @param wrist
     * @return
     */
    public static Command pickUpCube(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        return new SequentialCommandGroup(
                ShoulderCommands.setLowGoal(shoulder),
                ForearmCommands.extendForearm(forearm),
                WristCommands.intakeCubeCommand(wrist).withTimeout(Wrist.Map.kIntakeCubeTime),
                WristCommands.stopMotorsCommand(wrist),
                ForearmCommands.retractForearm(forearm));
    }

    // /**
    // * Intakes a game piece using the shoulder, wrist, and forearm.
    // *
    // * @param shoulder
    // * @param wrist
    // * @param forearm
    // * @param intakeDirection
    // * @return
    // */
    // public static Command intakeGamePiece(Shoulder shoulder, Wrist wrist, Forearm
    // forearm, IntakeDirection direction) {
    // return new SequentialCommandGroup(
    // WristCommands.setWristAngle(wrist, direction),
    // ShoulderCommands.setAngle(shoulder, Shoulder.Map.kGroundAngle),
    // // TODO: set forearm extend distance
    // WristCommands.runUntilTimeout(wrist, 3, direction == IntakeDirection.kCone
    // ? WristMap.kIntakeConeSpeed
    // : WristMap.kIntakeCubeSpeed));
    // }

    // /**
    // * Disables the shoulder PID and runs it manually, IF the forearm is retracted
    // *
    // * @param shoulder
    // * @param forearmIsRetractedSupplier
    // * @param joystickSupplier
    // * @return
    // */
    // public static Command raiseEffectorManually(Shoulder shoulder,
    // BooleanSupplier forearmIsRetractedSupplier,
    // DoubleSupplier joystickSupplier) {
    // return new SequentialCommandGroup(
    // Commands.runOnce(() -> shoulder.disable()),
    // Commands.run(() -> {
    // if (!forearmIsRetractedSupplier.getAsBoolean()) {
    // shoulder.setSpeed(0);
    // return;
    // }

    // shoulder.setSpeed(joystickSupplier.getAsDouble());
    // }, shoulder));
    // }

    // /**
    // * Score cube by setting the shoulder angle, forearm extension, and wrist
    // angle
    // * out, then ejecting for 1 second
    // *
    // * @param shoulder The shoulder of the robot
    // * @param wrist The wrist of the robot
    // * @param forearm The forearem of the robot
    // * @param level The desired shoulder level to score at
    // * @return
    // */
    // public static Command scoreCube(Shoulder shoulder, Wrist wrist, Forearm
    // forearm, ShoulderLevels level) {
    // return new SequentialCommandGroup(
    // ShoulderCommands.setAngle(shoulder, level),
    // // TODO: Set forearm extend distance
    // WristCommands.setWristAngle(wrist, IntakeDirection.kCone), // TODO: Verify
    // this is the right setting
    // WristCommands.runUntilTimeout(wrist, 1, WristMap.kEjectCubeSpeed),
    // WristCommands.stopIntake(wrist));
    // }

    // /**
    // * Score cone by setting the shoulder angle and forearm extension, then
    // ejecting
    // * for 1 second
    // *
    // * @param shoulder The shoulder of the robot
    // * @param wrist The wrist of the robot
    // * @param forearm The forearem of the robot
    // * @param level The desired shoulder level to score at
    // * @return
    // */
    // public static Command scoreCone(Shoulder shoulder, Wrist wrist, Forearm
    // forearm, ShoulderLevels level) {
    // return new SequentialCommandGroup(
    // ShoulderCommands.setAngle(shoulder, level),
    // // TODO: Set forearm extend distance
    // WristCommands.setWristAngle(wrist, IntakeDirection.kCone), // TODO: Verify
    // this is the right setting
    // WristCommands.runUntilTimeout(wrist, 1, WristMap.kEjectConeSpeed),
    // WristCommands.stopIntake(wrist));
    // }
}
