package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist;

public class WristCommands {

    public static HashMap<String, Command> getEvents(Wrist wrist) {
        return new HashMap<>() {
            {
                put("IntakeCube", runIntake(wrist, () -> true, () -> false));
                put("Eject", runIntake(wrist, () -> false, () -> true));
                put("intakeForTime", intakeForTimeCommand(wrist));
                put("ejectForTime", EjectForTimeCommand(wrist, 1));
            }
        };
    }

    public static Command runIntake(Wrist wrist, BooleanSupplier eject,
            BooleanSupplier intake) {
        return Commands.run(() -> {

            if (eject.getAsBoolean()) {
                wrist.runMotors(-1);
            } else if (intake.getAsBoolean()) {
                wrist.runMotors(0.6);

            } else {
                wrist.stopMotors();
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
        return Commands.run(() -> wrist.runMotors(Wrist.Map.kEjectSpeed));
    }

    public static Command stopMotorsCommand(Wrist wrist) {
        return Commands.run(() -> {
            wrist.stopMotors();
        });
    }

    public static Command intakeForTimeCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(0.6), wrist).withTimeout(0.5);
    }

    public static Command EjectForTimeCommand(Wrist wrist, double timeSeconds) {
        return Commands.run(() -> wrist.runMotors(1), wrist).withTimeout(timeSeconds);
    }

}
