package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.models.ShoulderLevels;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {
    public static Command setHighGoal(Shoulder shoulder) {
        return Commands.run(() -> {
            shoulder.extendShortSolenoid(true);
            shoulder.extendLongSolenoid(true);
        }, shoulder);
    }

    public static Command setMiddleGoal(Shoulder shoulder) {
        return Commands.run(() -> {
            shoulder.extendShortSolenoid(false);
            shoulder.extendLongSolenoid(true);
        }, shoulder);
    }

    public static Command setLowGoal(Shoulder shoulder) {
        return Commands.run(() -> {
            shoulder.extendLongSolenoid(false);
            shoulder.extendShortSolenoid(false);
        }, shoulder);
    }
}
