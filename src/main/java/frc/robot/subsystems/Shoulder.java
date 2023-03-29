package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.config.WristMap;
import frc.robot.models.ShoulderLevels;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;

public class Shoulder<K extends Number, V extends Number> extends PIDSubsystem {
    public static class Map {
        // CAN IDs & Channels
        public static final int kShoulder1Id = 21;
        public static final int kShoulder2Id = 23;
        public static final int kEncoderId = 20;
        public static final int kUpperLimitSwitchDIOChannel = 0;

        // Scoring angles
        public static final int kTopRowAngle = 68;
        public static final int kMiddleRowAngle = 42;
        public static final int kGroundAngle = 28;

        // PID
        public static final PidConstants kSprocketPid = new PidConstants((1d / 8), 0, 0.0012);

        // Constants
        public static final double kOpenLoopRampRate = 0.3;
        public static final double kLowerAngleLimit = 25;
        public static final double kUpperAngleLimit = 95;
        public static final double kHorizontalHoldOutput = -0.09 * 2;
        public static final double kMaxVelocityDegreesPer100ms = 2;
        public static final double kMaxAccelerationDegreesPer100ms = 0.25;
        public static final double kMaxSpeed = 0.6;
    }

    private LazyWPITalonSRX mShoulderMaster;
    private LazyWPITalonSRX shoulder2;
    private WPI_CANCoder mEncoder;
    private DigitalInput mUpperLimitSwitch;
    private Forearm _mForearm;
    private InterpolatingTreeMap<K, V> _mInterpolatedDMap;
    private double _lastOutput = 0;
    private double _lastFeedForward = 0;
    private double _lastFinalOutput = 0;

    public Shoulder(Forearm forearm) {
        super(new PIDController(Map.kSprocketPid.kP, Map.kSprocketPid.kI, Map.kSprocketPid.kD_min));
        getController().setTolerance(1);
        _mForearm = forearm;
        _mInterpolatedDMap.put(0, Map.kSprocketPid.kD_min);
        // _mInterpolatedDMap.put(Forearm.Map.kMaxDistanceOutSensorUnits,
        // Map.kSprocketPid.kD_max);

        mUpperLimitSwitch = new DigitalInput(Map.kUpperLimitSwitchDIOChannel);

        mEncoder = new WPI_CANCoder(Map.kEncoderId);
        mEncoder.clearStickyFaults();
        mEncoder.configFactoryDefault();
        mEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Setup the first shoulder motor
        mShoulderMaster = new LazyWPITalonSRX(Map.kShoulder1Id);
        mShoulderMaster.clearStickyFaults();
        mShoulderMaster.configFactoryDefault();

        // Define the motor's behavior
        mShoulderMaster.configOpenloopRamp(Map.kOpenLoopRampRate);
        mShoulderMaster.setNeutralMode(NeutralMode.Brake);
        mShoulderMaster.setInverted(InvertType.InvertMotorOutput);
        // mShoulderMaster.enableVoltageCompensation(true);
        mShoulderMaster.configContinuousCurrentLimit(50);
        mShoulderMaster.configPeakCurrentLimit(80);
        mShoulderMaster.configPeakCurrentDuration(150);

        shoulder2 = new LazyWPITalonSRX(Map.kShoulder2Id);
        shoulder2.clearStickyFaults();
        shoulder2.configFactoryDefault();
        shoulder2.setNeutralMode(NeutralMode.Brake);
        shoulder2.follow(mShoulderMaster);
        shoulder2.setInverted(InvertType.FollowMaster);
    }

    public void setAngle(double angleInDegrees) {
        if (!isEnabled())
            enable();

        setSetpoint(angleInDegrees);
    }

    public void setAngle(ShoulderLevels level) {
        if (!isEnabled())
            enable();

        switch (level) {
            default:
                setSetpoint(Shoulder.Map.kGroundAngle);
                break;
            case kSCORE_MIDDLE:
                setSetpoint(Shoulder.Map.kMiddleRowAngle);
                break;
            case kSCORE_HIGH:
                setSetpoint(Shoulder.Map.kTopRowAngle);
        }
    }

    public void setSpeed(double speed) {
        if (isEnabled())
            disable();

        runShoulderWithLimits(-1 * speed);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition());
    }

    public boolean isUpperSoftLimitReached() {
        return getMeasurement() >= Map.kUpperAngleLimit - 1;
    }

    public boolean isLowerLimitSwitchReached() {
        return getMeasurement() < Map.kLowerAngleLimit + 5;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        var gravCompensation = Math.cos(getRotation().getRadians());
        _lastOutput = output;
        _lastFeedForward = gravCompensation * Map.kHorizontalHoldOutput;
        _lastFinalOutput = MathUtil.clamp(output + _lastFeedForward, -1, 1);

        if (isEnabled()) {
            // adjust the D value based on how far extended the forearm is
            // _mForearm.getDistanceSensorUnits();
            // getController().setD();
        }

        runShoulderWithLimits(_lastFinalOutput);
    }

    @Override
    public double getMeasurement() {
        return getRotation().getDegrees();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Position", () -> getMeasurement(), null);
        builder.addDoubleProperty("Last motor output", () -> _lastOutput, null);
        builder.addBooleanProperty("Upper Limit Switch", mUpperLimitSwitch::get, null);
        builder.addBooleanProperty("Lower Limit Reached", this::isLowerLimitSwitchReached, null);
    }

    /**
     * Run shoulder at low speed with a deadband
     * 
     * @param speed
     */
    private void runShoulderWithLimits(double speed) {
        speed *= Map.kMaxSpeed;
        var clampedValue = MathUtil.clamp(MathUtil.applyDeadband(speed, 0.08), -Map.kMaxSpeed, Map.kMaxSpeed);
        _lastOutput = clampedValue;

        // Are we at our forward limit?
        var atSoftLimit = isUpperSoftLimitReached();
        var atHardLimit = mUpperLimitSwitch.get();
        if (clampedValue > 0 && (atSoftLimit || atHardLimit)) {
            mShoulderMaster.stopMotor();
            return;
        }

        // Are we at our reverse limit?
        if (clampedValue < 0 && isLowerLimitSwitchReached()) {
            mShoulderMaster.stopMotor();
            return;
        }

        if (clampedValue < 0)
            clampedValue *= 0.5;

        mShoulderMaster.set(ControlMode.PercentOutput, clampedValue);
    }
}
