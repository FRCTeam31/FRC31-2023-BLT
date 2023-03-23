package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;

public class Forearm extends SubsystemBase {
    public static class ForearmMap {
        // CAN IDs
        public static final int kForearmMotor1Id = 25;

        // PID
        public static final PidConstants PushAndPullPIDConstants = new PidConstants(0.04);

        // Constants
        public static final double kOpenLoopRampRate = 1.00;
        public static final double kDistancePerEncoderPulse = 0;
        public static final double kLowAngleLimit = 180;
        public static final double kHighAngleLimit = 220;
    }

    private LazyWPITalonSRX forearmMotor;
    private double _lastPidOutput = 0;
    private double _lastSetpoint = 0;

    public Forearm() {
        forearmMotor = new LazyWPITalonSRX(ForearmMap.kForearmMotor1Id);
        forearmMotor.clearStickyFaults();
        forearmMotor.configFactoryDefault();
        forearmMotor.setNeutralMode(NeutralMode.Brake);
        // forearmMotor.configOpenloopRamp(ForearmMap.kOpenLoopRampRate);
        forearmMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
        forearmMotor.setInverted(true);

        SmartDashboard.putData("Motor", forearmMotor);
    }

    public void setSetpoint(double distance) {
        _lastSetpoint = distance;
        // forearmMotor.set(ControlMode.MotionMagic, MathUtil.clamp(distance,
        // ForearmMap.kLowAngleLimit,
        // ForearmMap.kHighAngleLimit));
    }

    public void runSimple(double speed) {
        forearmMotor.set(ControlMode.PercentOutput, -speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Setpoint", () -> _lastSetpoint, (sp) -> setSetpoint(sp));
        builder.addDoubleProperty("PID output", () -> _lastPidOutput, null);
        builder.addDoubleProperty("Position", forearmMotor::getSelectedSensorPosition, null);
    }
}
