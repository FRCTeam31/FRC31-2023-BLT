package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.models.PidConstants;
import prime.movers.LazySolenoid;
import prime.movers.LazyWPITalonSRX;

public class Forearm extends SubsystemBase {

    public LazySolenoid forearmSolenoid;

    public static class Map {
        public static final int forearmSolenoidChannel = 0;
    }

    public Forearm() {
        forearmSolenoid = new LazySolenoid(PneumaticsModuleType.CTREPCM, Map.forearmSolenoidChannel);

    }

    public void extendForearmSolenoid(boolean extended) {
        forearmSolenoid.set(extended);
    }
}
