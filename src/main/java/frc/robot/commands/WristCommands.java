package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Wrist;

public class WristCommands {

    public static HashMap<String, Command> getEvents(Subsystem wrist) {
        return new HashMap<>() {
            {
                put("IntakeCube", runIntake((Wrist) wrist, () -> true, () -> false));
                // put("RunIntake", runIntake((Wrist) wrist, , null));
            }
        };
    }

    /***
     * Runs the intake depending on which trigger is pressed.
     * 
     * @param wrist
     * @param leftTriggerPressed
     * @param rightTriggerPressed
     * @return
     */
    public static Command runIntake(Wrist wrist, BooleanSupplier eject,
            BooleanSupplier intake) {
        return Commands.run(() -> {

            if (intake.getAsBoolean()) {
                wrist.runWrist1(Wrist.Map.kIntakeSpeed);
                wrist.runWrist2(Wrist.Map.kIntakeSpeed);
            } else if (eject.getAsBoolean()) {
                wrist.runWrist1(-Wrist.Map.kEjectSpeed);
                wrist.runWrist2(-Wrist.Map.kEjectSpeed);
            } else {
                wrist.stopMotors();
            }

        }, wrist);
    }

    /***
     * Command for ejecting a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command ejectCubeCommand(Wrist wrist) {
        return Commands.runOnce(() -> {
            wrist.runWrist1(-1);
            wrist.runWrist2(-1);
        }, wrist);

    }

    /***
     * Command for intaking a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command intakeCubeCommand(Wrist wrist) {
        return Commands.runOnce(() -> {
            wrist.runWrist1(1);
            wrist.runWrist2(1);
        }, wrist);
    }

    /***
     * Command for stopping the Motors.
     * 
     * @param wrist
     * @return
     */

    /***
     * Command for shooting a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command shootCubeCommand(Wrist wrist) {
        return Commands.run(() -> {
            wrist.runWrist1(Wrist.Map.kEjectSpeed);
            wrist.runWrist2(Wrist.Map.kEjectSpeed);
        }, wrist);
    }

    public static Command stopMotorsCommand(Wrist wrist) {
        return Commands.run(() -> {
            wrist.stopMotors();
        });
    }

}
