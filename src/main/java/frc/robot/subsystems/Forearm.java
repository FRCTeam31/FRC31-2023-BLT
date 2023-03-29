package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;

public class Forearm extends SubsystemBase {
    public static class Map {
        // CAN IDs & channels
        public static final int kForearmMotor1Id = 25;
        public static final int kRevLimitSwInputChannel = 1;

        // PID
        public static final PidConstants PushAndPullPIDConstants = new PidConstants(0.04);

        // Constants
        public static final double kOpenLoopRampRate = 1.00;
        public static final double kDistancePerEncoderPulse = 0;
        public static final double kLowAngleLimit = 180;
        public static final double kHighAngleLimit = 220;
        public static double _forearmRotationCircumference = 1;
        public static double _forearmGearRatio = 1;

        public static final double kMaxDistanceOutSensorUnits = 31000;
        public static final double kMinDistanceOutSensorUnits = 3000;

        public static final double kPickUpGroundDistance = 0;
    }

    private LazyWPITalonSRX forearmMotor;
    private DigitalInput mReverseLimitSwitch;
    private double _lastOutput = 0;
    private double _currentEncoderZeroOffset = 0;

    public Forearm() {
        forearmMotor = new LazyWPITalonSRX(Map.kForearmMotor1Id);
        forearmMotor.clearStickyFaults();
        forearmMotor.configFactoryDefault();
        forearmMotor.setNeutralMode(NeutralMode.Coast);
        // forearmMotor.configOpenloopRamp(ForearmMap.kOpenLoopRampRate);
        forearmMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
        forearmMotor.setInverted(false);
        forearmMotor.setSensorPhase(true);

        mReverseLimitSwitch = new DigitalInput(Map.kRevLimitSwInputChannel);
    }

    public void runSimple(double speed) {
        _lastOutput = speed;

        if (speed < 0 && getMinSoftLimitReached()) {
            forearmMotor.stopMotor();
            return;
        }

        if (speed > 0 && getMaxSoftLimitReached()) {
            forearmMotor.stopMotor();
            return;
        }

        forearmMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void home() {
        while (!mReverseLimitSwitch.get()) {
            forearmMotor.set(ControlMode.PercentOutput, 0.5);
        }

        resetEncoderOffset();
    }

    public void resetEncoderOffset() {
        _currentEncoderZeroOffset = getDistanceSensorUnits();
    }

    public double getDistanceSensorUnits() {
        return -forearmMotor.getSelectedSensorPosition();
    }

    public boolean getMaxSoftLimitReached() {
        return getDistanceSensorUnits() >= _currentEncoderZeroOffset + Map.kMaxDistanceOutSensorUnits;
    }

    public boolean getMinSoftLimitReached() {
        return getDistanceSensorUnits() <= _currentEncoderZeroOffset + Map.kMinDistanceOutSensorUnits;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addDoubleProperty("Setpoint", () -> _lastSetpoint, (sp) ->
        // setSetpoint(sp));
        builder.addDoubleProperty("Motor output", () -> _lastOutput, null);
        builder.addDoubleProperty("Position", this::getDistanceSensorUnits, null);
        builder.addDoubleProperty("Encoder offset", () -> _currentEncoderZeroOffset, null);
        builder.addBooleanProperty("Max soft limit reached", this::getMaxSoftLimitReached, null);
        builder.addBooleanProperty("Min soft limit reached", this::getMinSoftLimitReached, null);
        builder.addBooleanProperty("Reverse Limit Switch", mReverseLimitSwitch::get, null);
    }
}
