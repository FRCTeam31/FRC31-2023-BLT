package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.WristMap;
import frc.robot.subsystems.Wrist;

public class WristCommands {
    public static Command runIntakeConeAndEjectCubeCommand(Wrist wrist, boolean up) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(up), wrist),
                Commands.run(() -> wrist.runMotors(-WristMap.kEjectConeSpeed), wrist)
        });
    }

    public static Command runIntakeCubeAndEjectConeCommand(Wrist wrist, boolean up) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(up), wrist),
                Commands.run(() -> wrist.runMotors(WristMap.kIntakeCubeSpeed), wrist)
        });
    }

    public static Command runIntakeFallenConeCommand(Wrist wrist, boolean up) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(false), wrist),
                Commands.run(() -> wrist.runMotors(-WristMap.kIntakeConeSpeed), wrist)
        });
    }

    public static Command stopIntakeCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.stopIntake(), wrist);
    }

    // Test commands
    public static Command runMotorSimpleCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(0.5), wrist);
    }

    public static Command toggleActuatorCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.setWrist(!wrist.getWristOut()), wrist);
    }
}
