package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;
import prime.utilities.CTREConverter;

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

        SmartDashboard.putData("Forearm Motor", forearmMotor);
    }

    public void setSetpoint(double angle) {
        _lastSetpoint = angle;
        forearmMotor.set(ControlMode.MotionMagic, MathUtil.clamp(angle, ForearmMap.kLowAngleLimit,
                ForearmMap.kHighAngleLimit));
    }

    public void runSimple(double speed) {
        forearmMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Shoulder Setpoint", () -> _lastSetpoint, (sp) -> setSetpoint(sp));
        builder.addDoubleProperty("Shoulder last PID output", () -> _lastPidOutput, null);
        builder.addDoubleProperty("Shoulder position", () -> forearmMotor.getSelectedSensorPosition(), null);
    }
}
