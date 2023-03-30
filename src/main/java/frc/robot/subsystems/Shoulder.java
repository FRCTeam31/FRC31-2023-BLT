package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazySolenoid;

public class Shoulder extends SubsystemBase {

    private LazySolenoid shortSolenoid;
    private LazySolenoid longSolenoid;

    public static class Map {
        public static final int shortSolenoidChannel = 0;
        public static final int longSolenoidChannel = 0;
    }

    public Shoulder() {
        shortSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.shortSolenoidChannel);
        longSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.longSolenoidChannel);
    }

    public void extendShortSolenoid(boolean extended) {
        shortSolenoid.set(extended);
    }

    public void extendLongSolenoid(boolean extended) {
        longSolenoid.set(extended);
    }

}
