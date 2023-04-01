package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;

public class Wrist extends SubsystemBase {

    public LazyCANSparkMax wrist1;
    public LazyCANSparkMax wrist2;

    /***
     * Contains the constants for the wrist.
     */
    public class Map {
        public static final int kWrist1CanId = 22;
        public static final int kWrist2CanId = 24;
        public static final double kIntakeSpeed = 0.4;
        public static final double kEjectSpeed = 1;
        public static final double kTriggerDeadband = 0.1;
        public static final double kEjectCubeTime = 1;
        public static final double kIntakeCubeTime = 1;
    }

    /***
     * Wrist.
     */
    public Wrist() {
        wrist1 = new LazyCANSparkMax(Map.kWrist1CanId, MotorType.kBrushless);
        wrist2 = new LazyCANSparkMax(Map.kWrist2CanId, MotorType.kBrushless);

    }

    /***
     * Sets the motorspeed for the wrists.
     * 
     * @param motorSpeed
     */
    public void runMotors(double motorSpeed) {
        wrist1.set(motorSpeed);
        wrist2.set(-motorSpeed);
    }

    @Override
    /***
     * Smart Dashboard so smart. and Dashboard.
     */
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

    public void stopMotors() {
        wrist1.stopMotor();
        wrist2.stopMotor();
    }

}
