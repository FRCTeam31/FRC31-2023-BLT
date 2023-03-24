package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.WristMap;
import frc.robot.models.IntakeDirection;
import frc.robot.subsystems.Wrist;

public class WristCommands {
    public static Command runIntake(Wrist wrist, BooleanSupplier leftTriggerPressed,
            BooleanSupplier rightTriggerPressed) {
        return Commands.run(() -> {
            if (leftTriggerPressed.getAsBoolean()) {
                wrist.runMotors(WristMap.kEjectConeSpeed);
                return;
            } else if (rightTriggerPressed.getAsBoolean()) {
                wrist.runMotors(WristMap.kIntakeConeSpeed);
                return;
            } else {
                wrist.runMotors(0);
            }

        }, wrist);
    }

    public static Command setWristAngle(Wrist wrist, IntakeDirection intakeDirection) {
        return Commands.runOnce(() -> {
            wrist.setWrist(intakeDirection == IntakeDirection.kCone);
        });
    }

    public static Command runIntakeFallenConeCommand(Wrist wrist) {
        return new SequentialCommandGroup(new Command[] {
                Commands.runOnce(() -> wrist.setWrist(false), wrist),
                Commands.run(() -> wrist.runMotors(-WristMap.kIntakeConeSpeed), wrist)
        });
    }

    public static Command stopIntake(Wrist wrist) {
        return Commands.runOnce(() -> wrist.stopIntake(), wrist);
    }

    public static Command runUntilTimeout(Wrist wrist, double seconds, double speed) {
        return Commands.run(() -> {
            wrist.runMotors(speed);
        }).withTimeout(seconds);
    }

}
