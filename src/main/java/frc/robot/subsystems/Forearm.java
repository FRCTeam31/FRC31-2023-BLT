package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Forearm extends SubsystemBase {

    public DoubleSolenoid forearmSolenoid;

    /***
     * Contains constants for the Forearm.
     */
    public static class Map {
        public static final int forearmSolenoidForwardChannel = 0;
        public static final int forearmSolenoidReverseChannel = 1;
    }

    /***
     * Forearm.
     */
    public Forearm() {
        forearmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Map.forearmSolenoidForwardChannel,
                Map.forearmSolenoidReverseChannel);
    }

    /***
     * Extends the Forearm.
     * 
     * @param extended
     */
    public void extendForearmSolenoid(boolean extended) {
        if (extended) {
            forearmSolenoid.set(Value.kForward);
        } else {
            forearmSolenoid.set(Value.kReverse);
        }
    }

    public boolean forearmIsExtended() {
        if (forearmSolenoid.get() == Value.kReverse) {
            return false;
        } else {
            return true;
        }

    }
}