package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.config.WristMap;
import frc.robot.models.IntakeDirection;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class WristCommands {

    public static HashMap<String, Command> getEvents(Wrist wrist) {
        return new HashMap<>() {
            {
                put("IntakeCube", runIntake(wrist, () -> true, () -> false));
                put("Eject", runIntake(wrist, () -> false, () -> true));
            }
        };
    }

    public static Command runIntake(Wrist wrist, BooleanSupplier eject,
            BooleanSupplier intake) {
        return Commands.run(() -> {

            if (eject.getAsBoolean()) {
                wrist.runMotors(-Wrist.Map.kIntakeSpeed);
            } else if (intake.getAsBoolean()) {
                wrist.runMotors(Wrist.Map.kIntakeSpeed);

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

}
