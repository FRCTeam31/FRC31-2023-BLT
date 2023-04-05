package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

    private DoubleSolenoid shortSolenoid;
    private DoubleSolenoid longSolenoid;

    /***
     * Contains the constants for the Shoulder
     */
    public static class Map {
        public static final int shortSolenoidForwardChannel = 4;
        public static final int shortSolenoidReverseChannel = 5;
        public static final int longSolenoidForwardChannel = 2;
        public static final int longSolenoidReverseChannel = 3;
    }

    /***
     * Shoulder Constructor
     */
    public Shoulder() {
        shortSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Map.shortSolenoidForwardChannel,
                Map.shortSolenoidReverseChannel);
        longSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Map.longSolenoidForwardChannel,
                Map.longSolenoidReverseChannel);
        shortSolenoid.set(Value.kForward);
    }

    /***
     * Extends the small piston of the Forearm.
     * 
     * @param extended
     */
    public void extendShortSolenoid(boolean extended) {
        if (extended) {
            shortSolenoid.set(Value.kForward);
        } else {
            shortSolenoid.set(Value.kReverse);
        }
    }

    /***
     * Extends the long piston of the Forearm.
     * 
     * @param extended
     */
    public void extendLongSolenoid(boolean extended) {
        if (extended) {
            longSolenoid.set(Value.kForward);
        } else {
            longSolenoid.set(Value.kReverse);
        }
    }

    public boolean isFullyRetracted() {

        if (shortSolenoid.get() == Value.kReverse && longSolenoid.get() == Value.kReverse) {
            return true;
        }
        return false;

    }

    public String detectCurrentArmPosition() {

        if (longSolenoid.get() == Value.kForward && shortSolenoid.get() == Value.kForward) {
            return "High Goal";
        } else if (longSolenoid.get() == Value.kReverse && shortSolenoid.get() == Value.kForward) {
            return "Low Goal";
        } else {
            return "Middle Goal";
        }
    }
    // This is added becuase we were seeing loop overun errors coming from the base
    // method

    @Override
    public void periodic() {

    }
}
