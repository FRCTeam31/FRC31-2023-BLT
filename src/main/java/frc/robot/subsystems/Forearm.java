package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.models.PidConstants;
import prime.movers.LazySolenoid;
import prime.movers.LazyWPITalonSRX;

public class Forearm extends SubsystemBase {

    public DoubleSolenoid forearmSolenoid;

    /***
     * Contains constants for the Forearm.
     */
    public static class Map {
        public static final int forearmSolenoidForwardChannel = 1;
        public static final int forearmSolenoidReverseChannel = 0;
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
        } else if (!extended) {
            forearmSolenoid.set(Value.kReverse);
        }
    }
}
