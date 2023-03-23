package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;
import prime.utilities.CTREConverter;

public class Shoulder extends PIDSubsystem {
    public static class Map {
        // CAN IDs & Channels
        public static final int kShoulder1Id = 21;
        public static final int kShoulder2Id = 23;
        public static final int kEncoderId = 20;
        public static final int kUpperLimitSwitchDIOChannel = 0;

        // PID
        public static final String kSprocketPidName = "Shoulder PID constants";
        public static final PidConstants kSprocketPid = new PidConstants((1d / 80), 0.002, 0.0005);

        // Constants
        public static final double kOpenLoopRampRate = 1.00;
        public static final double kLowAngleLimit = 5.5;
        public static final double kHorizontalHoldOutput = -0.09;
        public static final double kMaxVelocityDegreesPer100ms = 2;
        public static final double kMaxAccelerationDegreesPer100ms = 0.25;
        public static final double kMaxSpeed = 0.6;

        // Scoring angles
        public static final int kTopRowAngle = 120;
        public static final int kMiddleRowAngle = 50;
        public static final int kGroundLevelAngle = 10;
    }

    private LazyWPITalonSRX mShoulderMaster;
    private LazyWPITalonSRX shoulder2;
    private WPI_CANCoder mEncoder;
    private DigitalInput mUpperLimitSwitch;
    private double _lastOutput = 0;
    private double _lastFeedForward = 0;
    private double _lastFinalOutput = 0;

    public Shoulder() {
        super(new PIDController(Map.kSprocketPid.kP, Map.kSprocketPid.kI, Map.kSprocketPid.kD));
        getController().setTolerance(1);

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
        mShoulderMaster.setNeutralMode(NeutralMode.Brake);
        mShoulderMaster.setInverted(InvertType.InvertMotorOutput);
        mShoulderMaster.configContinuousCurrentLimit(20);
        mShoulderMaster.configPeakCurrentLimit(30);
        mShoulderMaster.configPeakCurrentDuration(250);

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

    public void setSpeed(double speed) {
        if (isEnabled())
            disable();

        runShoulderWithLimits(-1 * speed);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition());
    }

    public boolean getLowerLimitSwitchReached() {
        return getMeasurement() < Map.kLowAngleLimit + 5;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        var gravCompensation = Math.cos(getRotation().getRadians());
        _lastOutput = output;
        _lastFeedForward = gravCompensation * Map.kHorizontalHoldOutput;
        _lastFinalOutput = MathUtil.clamp(output + _lastFeedForward, -1, 1);

        runShoulderWithLimits(_lastFinalOutput);
    }

    @Override
    protected double getMeasurement() {
        return getRotation().getDegrees();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> getMeasurement(), null);
        builder.addDoubleProperty("Last motor output", () -> _lastOutput, null);
        builder.addBooleanProperty("Upper Limit Switch", mUpperLimitSwitch::get, null);
        builder.addBooleanProperty("Lower Limit Reached", this::getLowerLimitSwitchReached, null);
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
        if (clampedValue > 0 && mUpperLimitSwitch.get())
            return;

        // Are we at our reverse limit?
        if (clampedValue < 0 && getLowerLimitSwitchReached())
            return;

        mShoulderMaster.set(ControlMode.PercentOutput, clampedValue);
    }
}
