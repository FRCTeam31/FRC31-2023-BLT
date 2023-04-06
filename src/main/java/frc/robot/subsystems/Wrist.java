package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;

public class Wrist extends SubsystemBase {
    // public DigitalInput innerLimitSwitch;
    public LazyCANSparkMax wrist1;
    public LazyCANSparkMax wrist2;

    /***
     * Contains the constants for the wrist.
     */
    public static class WristMap {
        public static final int kWrist1CanId = 22;
        public static final int kWrist2CanId = 24;
        public static final double kIntakeSpeed = 0.6;
        public static final double kEjectSpeed = 1;
        public static final double kTriggerDeadband = 0.1;
        public static final double kEjectCubeTime = 1;
        public static final double kIntakeCubeTime = 1;
        // public static final int kIntakeLimitSwitchChannel = 9;
    }

    /***
     * Wrist.
     */
    public Wrist() {
        wrist1 = new LazyCANSparkMax(WristMap.kWrist1CanId, MotorType.kBrushless);
        wrist2 = new LazyCANSparkMax(WristMap.kWrist2CanId, MotorType.kBrushless);
        // innerLimitSwitch = new DigitalInput(WristMap.kIntakeLimitSwitchChannel);
    }

    public void runMotors(double speed) {
        // if (speed < 0 && innerLimitSwitch.get()) {
        // stopMotors();
        // }

        wrist1.set(speed);
        wrist2.follow(wrist1);
    }

    /***
     * Stops the motors (crazy)
     */
    public void stopMotors() {
        wrist1.stopMotor();
        wrist2.stopMotor();
    }

    /***
     * Smart Dashboard so smart. and Dashboard.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
