package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.models.ShoulderLevels;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {

    /***
     * Sets the Height of the Forearm to the High Goal.
     * 
     * @param shoulder
     * @return
     */
    public static Command setHighGoal(Shoulder shoulder) {
        return Commands.runOnce(() -> {
            shoulder.extendShortSolenoid(true);
            shoulder.extendLongSolenoid(true);
        }, shoulder);
    }

    /***
     * Sets the Height of the Forearm to the Middle Goal.
     * 
     * @param shoulder
     * @return
     */
    public static Command setMiddleGoal(Shoulder shoulder) {
        return Commands.runOnce(() -> {
            shoulder.extendShortSolenoid(false);
            shoulder.extendLongSolenoid(true);
        }, shoulder);
    }

    /***
     * Turns on the Robots autonomous kill mode. Erm, I mean, sets the forearm to
     * the Low Goal.
     * 
     * @param shoulder
     * @return
     */
    public static Command setLowGoal(Shoulder shoulder) {
        return Commands.runOnce(() -> {

            shoulder.extendLongSolenoid(false);
            shoulder.extendShortSolenoid(true);

        }, shoulder);

    }

    // public static Command setGroundRetractedCommand(Shoulder shoulder, Forearm
    // forearm) {

    // if(forearm.forearmIsExtended() == Value.kReverse){
    // return new SequentialCommandGroup(
    // Commands.runOnce(() -> shoulder.extendLongSolenoid(false), null)
    // );
    // }

    // }

    public static Command setGround(Shoulder shoulder) {
        return Commands.runOnce(() -> {
            shoulder.extendLongSolenoid(false);
            shoulder.extendShortSolenoid(false);
        }, shoulder);
    }

}
