package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazySolenoid;

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

    public Shoulder() {
        shortSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Map.shortSolenoidForwardChannel,
                Map.shortSolenoidReverseChannel);
        longSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Map.longSolenoidForwardChannel,
                Map.longSolenoidReverseChannel);
    }

    /***
     * Extends the small piston of the Forearm.
     * 
     * @param extended
     */
    public void extendShortSolenoid(boolean extended) {
        if (extended) {
            shortSolenoid.set(Value.kForward);
        } else if (!extended) {
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
        } else if (!extended) {
            longSolenoid.set(Value.kReverse);
        }
    }
}
