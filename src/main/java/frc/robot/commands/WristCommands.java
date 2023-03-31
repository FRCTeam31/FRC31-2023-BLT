package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.ControlsMap;
import frc.robot.config.WristMap;
import frc.robot.models.IntakeDirection;
import frc.robot.subsystems.Wrist;

public class WristCommands {

    /***
     * Runs the intake depending on which trigger is pressed.
     * 
     * @param wrist
     * @param leftTriggerPressed
     * @param rightTriggerPressed
     * @return
     */
    public static Command runIntakeWithJoystickCommand(Wrist wrist, BooleanSupplier leftTriggerPressed,
            BooleanSupplier rightTriggerPressed) {
        return Commands.run(() -> {

            if (rightTriggerPressed.getAsBoolean()) {
                wrist.runMotors(Wrist.Map.kWristSpeed);
            }

            if (leftTriggerPressed.getAsBoolean()) {
                wrist.runMotors(-Wrist.Map.kWristSpeed);
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
        return Commands.run(() -> wrist.runMotors(-1));
    }

    /***
     * Command for intaking a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command intakeCubeCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(1));
    }

    /***
     * Command for stopping the Motors.
     * 
     * @param wrist
     * @return
     */
    public static Command stopMotorsCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(0));
    }

    /***
     * Command for shooting a Cube.
     * 
     * @param wrist
     * @return
     */
    public static Command shootCubeCommand(Wrist wrist) {
        return Commands.run(() -> wrist.runMotors(Wrist.Map.kShootCubeSpeed));
    }

}
