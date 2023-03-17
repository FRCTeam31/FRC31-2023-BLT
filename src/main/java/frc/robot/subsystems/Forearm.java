package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;

public class Forearm extends PIDSubsystem {
    public static class ForearmMap {
        // CAN IDs
        public static final int kForearmMotor1Id = 23;

        // PID
        public static final PidConstants PushAndPullPIDConstants = new PidConstants(0.04);

        // Constants
        public static final double kOpenLoopRampRate = 1.00;
        public static final double kDistancePerEncoderPulse = 0;
    }

    private LazyWPITalonSRX forearmMotor;
    private double _lastPidOutput = 0;
    private double _lastSetpoint = 0;
    private double _lastEncoderRead = 0;

    public Forearm() {
        super(new PIDController(ForearmMap.PushAndPullPIDConstants.kP, ForearmMap.PushAndPullPIDConstants.kI,
                ForearmMap.PushAndPullPIDConstants.kD));
        forearmMotor = new LazyWPITalonSRX(ForearmMap.kForearmMotor1Id);
        forearmMotor.clearStickyFaults();
        forearmMotor.configFactoryDefault();
        forearmMotor.setNeutralMode(NeutralMode.Brake);
        forearmMotor.configOpenloopRamp(ForearmMap.kOpenLoopRampRate);
        forearmMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

        enable();
    }

    public void setSetpoint(double angle) {
        if (!isEnabled())
            enable();

        _lastSetpoint = angle;
        super.setSetpoint(angle);
    }

    public void run(double speed) {
        if (isEnabled())
            disable();

        forearmMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        _lastPidOutput = output;
        // TODO: use the PID output below
        // forearmMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
        _lastEncoderRead = forearmMotor.getSelectedSensorPosition();

        return _lastEncoderRead;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Shoulder Setpoint", () -> _lastSetpoint, (sp) -> setSetpoint(sp));
        builder.addDoubleProperty("Shoulder last PID output", () -> _lastPidOutput, null);
        builder.addDoubleProperty("Shoulder last position", () -> _lastEncoderRead, null);
    }
}
