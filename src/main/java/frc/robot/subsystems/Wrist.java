package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;
import prime.movers.LazyWPITalonSRX;

public class Wrist extends SubsystemBase {
    /**
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
    }

    public LazyCANSparkMax wristLeader;
    public LazyWPITalonSRX wristFollower;
    private double _lastSpeed = -10;

    /**
     * Wrist.
     */
    public Wrist() {
        setName("Wrist");
        wristLeader = new LazyCANSparkMax(WristMap.kWrist1CanId, MotorType.kBrushless);
        wristFollower = new LazyWPITalonSRX(WristMap.kWrist2CanId);
    }

    /**
     * Sets the duty cycle of the motors in raw axis magnitude [-1,1]
     */
    public void runMotors(double speed) {
        _lastSpeed = speed;
        wristLeader.set(speed);
        wristFollower.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Stops the motors (crazy)
     */
    public void stopMotors() {
        _lastSpeed = 0;
        wristLeader.stopMotor();
        wristFollower.stopMotor();
    }

    /**
     * Smart Dashboard so smart. and Dashboard.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Output %", () -> _lastSpeed, null);
    }
}
