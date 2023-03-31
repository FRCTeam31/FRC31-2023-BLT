package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazySolenoid;

public class Shoulder extends SubsystemBase {

    private LazySolenoid shortSolenoid;
    private LazySolenoid longSolenoid;

    /***
     * Contains the constants for the Shoulder
     */
    public static class Map {
        public static final int shortSolenoidChannel = 0;
        public static final int longSolenoidChannel = 0;
    }

    /***
     * Shoulder
     */
    public Shoulder() {
        shortSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.shortSolenoidChannel);
        longSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.longSolenoidChannel);

        extendShortSolenoid(true);
    }

    /***
     * Extends the small piston of the Forearm.
     * 
     * @param extended
     */
    public void extendShortSolenoid(boolean extended) {
        shortSolenoid.set(extended);
    }

    /***
     * Extends the long piston of the Forearm.
     * 
     * @param extended
     */
    public void extendLongSolenoid(boolean extended) {
        longSolenoid.set(extended);
    }
}
