package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import prime.models.PidConstants;
import prime.movers.LazyWPITalonSRX;

public class Shoulder extends PIDSubsystem {
    public static class Map {
        // CAN IDs & Channels
        public static final int kShoulder1Id = 21;
        public static final int kShoulder2Id = 23;
        public static final int kEncoderId = 20;
        public static final int kUpperLimitSwitchDIOChannel = 0;

        // PID
        public static final String kSprocketPidName = "Shoulder PID constants";
        public static final PidConstants kSprocketPid = new PidConstants((1d / 160d), 0.008, 0);

        // Constants
        public static final double kOpenLoopRampRate = 1.00;
        public static final double kMaxAngle = 220;
        public static final double kMinAngle = 180;
        public static final double kmaxVelocityPer100ms = 10;
        public static final double kmaxAccelerationPer100ms = 10;
        public static final double kLowAngleLimit = 180;
        public static final double kHighAngleLimit = 220;
        public static final double kHorizontalHoldOutput = -0.09;
    }

    private LazyWPITalonSRX shoulder1;
    private LazyWPITalonSRX shoulder2;
    private WPI_CANCoder mEncoder;
    private DigitalInput mUpperLimitSwitch;
    private double _lastOutput = 0;
    private double _lastFeedForward = 0;
    private double _lastFinalOutput = 0;

    public Shoulder() {
        super(new PIDController(Map.kSprocketPid.kP, Map.kSprocketPid.kI, Map.kSprocketPid.kD));
        getController().setTolerance(1);
        SmartDashboard.putData("Shoulder PID Controller", getController());
        mUpperLimitSwitch = new DigitalInput(Map.kUpperLimitSwitchDIOChannel);

        shoulder1 = new LazyWPITalonSRX(Map.kShoulder1Id);
        shoulder1.clearStickyFaults();
        shoulder1.configFactoryDefault();
        shoulder1.setNeutralMode(NeutralMode.Brake);
        shoulder1.setInverted(InvertType.InvertMotorOutput);
        shoulder1.configMotionAcceleration(Map.kmaxAccelerationPer100ms);
        shoulder1.configMotionCruiseVelocity(Map.kmaxVelocityPer100ms);

        // PID
        shoulder1.config_kP(0, Map.kSprocketPid.kP);

        SmartDashboard.putData(shoulder1);

        shoulder2 = new LazyWPITalonSRX(Map.kShoulder2Id);
        shoulder2.clearStickyFaults();
        shoulder2.configFactoryDefault();
        shoulder2.setNeutralMode(NeutralMode.Brake);
        shoulder2.follow(shoulder1);
        shoulder2.setInverted(InvertType.FollowMaster);

        mEncoder = new WPI_CANCoder(Map.kEncoderId);
        mEncoder.clearStickyFaults();
        mEncoder.configFactoryDefault();
        mEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        SmartDashboard.putData(mEncoder);

        enable();
        setSetpoint(200);
    }

    /**
     * Run shoulder at low speed with a deadband
     * 
     * @param speed
     */
    public void runShoulder(double speed) {
        // if (isEnabled())
        // disable();

        if (speed > 0 && mUpperLimitSwitch.get())
            return;

        shoulder1.set(MathUtil.applyDeadband(speed, 0.15));
    }

    public void setShoulderAngle(double angleInDegrees) {
        if (!isEnabled())
            enable();

        setSetpoint(angleInDegrees);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition());
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        var gravCompensation = Math.cos(getRotation().getRadians());
        _lastOutput = output;
        _lastFeedForward = gravCompensation * Map.kHorizontalHoldOutput;
        _lastFinalOutput = MathUtil.clamp(output + _lastFeedForward, -1, 1);

        runShoulder(_lastFinalOutput);
    }

    @Override
    protected double getMeasurement() {
        return getRotation().getDegrees();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Shoulder Position", () -> mEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("PID setpoint", () -> getController().getSetpoint(), null);
        builder.addDoubleProperty("PID error", () -> getController().getPositionError(), null);
        builder.addDoubleProperty("FF", () -> _lastFeedForward, null);
        builder.addDoubleProperty("Final calculated output", () -> _lastFinalOutput, null);
        builder.addDoubleProperty("PId output + FF", () -> _lastOutput, null);
        builder.addBooleanProperty("Upper Limit Switch", mUpperLimitSwitch::get, null);
    }
}
