package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shoulder;

public class ShoulderCommands {

    public static HashMap<String, Command> getEvents(Shoulder shoulder) {
        return new HashMap<>() {
            {
                put("setHighGoal", setHighGoal(shoulder));
                put("setMidGoal", setMiddleGoal(shoulder));
                put("setLowGoal", setLowGoal(shoulder));
                put("setGround", setGround(shoulder));

            }
        };
    }

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
        }, shoulder).andThen(Commands.waitSeconds(0.75));
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
        }, shoulder).andThen(Commands.waitSeconds(0.75));
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

        }, shoulder).andThen(Commands.waitSeconds(0.75));

    }

    public static Command setGround(Shoulder shoulder) {
        return Commands.runOnce(() -> {
            shoulder.extendLongSolenoid(false);
            shoulder.extendShortSolenoid(false);
        }, shoulder).andThen(Commands.waitSeconds(0.75));
    }
}
