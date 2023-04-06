package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristMap;

public class WristCommands {

    public static HashMap<String, Command> getEvents(Wrist wrist) {
        return new HashMap<>() {
            {
                put("IntakeCube", runIntake(wrist, () -> true, () -> 0.6));
                put("Eject", runIntake(wrist, () -> false, () -> 0));
                put("intakeForTime", intakeForTimeCommand(wrist));
                put("ejectCube", EjectForTimeCommand(wrist, 0.75));
            }
        };
    }

    public static Command runIntake(Wrist wrist, BooleanSupplier eject, DoubleSupplier intake) {
        return Commands.run(() -> {

            if (eject.getAsBoolean()) {
                wrist.runMotors(-1);
            } else {
                var intakeSpeed = intake.getAsDouble();
                wrist.runMotors(intakeSpeed);
            }

        }, wrist);
    }

    /***
     * Runs the intake depending on which trigger is pressed.
     * 
     * @param wrist
     * @param leftTriggerPressed
     * @param rightTriggerPressed
     * @return
     */

    /***
     * Command for ejecting a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command ejectCubeCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(-1));
    }

    /***
     * Command for intaking a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command intakeCubeCommand(Wrist wrist) {
        return Commands.runOnce(() -> wrist.runMotors(1));
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
        return Commands.run(() -> wrist.runMotors(WristMap.kEjectSpeed));
    }

    public static Command stopMotorsCommand(Wrist wrist) {
        return Commands.runOnce(() -> {
            wrist.stopMotors();
        });
    }

    public static Command intakeForTimeCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(0.6), wrist).withTimeout(0.5);
    }

    public static Command EjectForTimeCommand(Wrist wrist, double timeSeconds) {
        return Commands.run(() -> wrist.runMotors(-1), wrist).withTimeout(timeSeconds)
                .andThen(stopMotorsCommand(wrist));
    }

}
