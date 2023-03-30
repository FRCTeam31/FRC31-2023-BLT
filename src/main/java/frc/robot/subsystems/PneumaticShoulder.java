package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazySolenoid;

public class PneumaticShoulder extends SubsystemBase {
    private LazySolenoid longSolenoid;
    private LazySolenoid shortSolenoid;

    public static class Map {

        public static final int longSolenoidChannel = 0;
        public static final int shortSolenoidChannel = 0;

    }

    public PneumaticShoulder() {
        longSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.longSolenoidChannel);
        shortSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.shortSolenoidChannel);

    }

    public void extendShortSolenoid(boolean extended) {
        shortSolenoid.set(extended);

    }

    public void extendLongSolenoid(boolean extended) {
        longSolenoid.set(extended);
    }

}
