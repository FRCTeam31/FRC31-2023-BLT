package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.config.DriveMap;
import prime.utilities.CTREConverter;
import prime.movers.LazyWPITalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SwerveModule extends PIDSubsystem {
    private LazyWPITalonFX mSteeringMotor;
    private LazyWPITalonFX mDriveMotor;
    private WPI_CANCoder mEncoder;
    private int mEncoderOffset;
    private SupplyCurrentLimitConfiguration mSupplyCurrentConfig = new SupplyCurrentLimitConfiguration(true, 50, 80,
            0.15);

    public SwerveModule(
            int driveMotorId,
            int steeringMotorId,
            int encoderId,
            int encoderAbsoluteOffset,
            boolean driveInverted) {
        super(new PIDController(DriveMap.kSteeringPidConstants.kP, DriveMap.kSteeringPidConstants.kI,
                DriveMap.kSteeringPidConstants.kD_min));
        mEncoderOffset = encoderAbsoluteOffset;

        // Set up the steering motor
        setupSteeringMotor(steeringMotorId);

        // Set up the drive motor
        setupDriveMotor(driveMotorId, driveInverted);

        // Set up our encoder
        mEncoder = new WPI_CANCoder(encoderId);
        mEncoder.clearStickyFaults();
        mEncoder.configFactoryDefault();
        mEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Create a PID controller to calculate steering motor output
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Math.PI / 1800);
        enable();
    }

    private void setupSteeringMotor(int steeringId) {
        mSteeringMotor = new LazyWPITalonFX(steeringId);
        mSteeringMotor.configFactoryDefault();
        mSteeringMotor.clearStickyFaults();
        mSteeringMotor.setNeutralMode(NeutralMode.Brake);
        mSteeringMotor.setInverted(TalonFXInvertType.CounterClockwise);
        // mSteeringMotor.configStatorCurrentLimit(statorCurrentConfig);
        mSteeringMotor.configSupplyCurrentLimit(mSupplyCurrentConfig);
    }

    public void setupDriveMotor(int driveMotorId, boolean driveInverted) {
        mDriveMotor = new LazyWPITalonFX(driveMotorId);

        mDriveMotor.configFactoryDefault();
        mDriveMotor.clearStickyFaults();
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.slot0.kP = 0.01;
        driveMotorConfig.slot0.kF = 0.48;
        driveMotorConfig.slot0.closedLoopPeriod = 100;
        driveMotorConfig.slot0.allowableClosedloopError = 250;

        mDriveMotor.configAllSettings(driveMotorConfig);
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.setInverted(driveInverted ? TalonFXInvertType.CounterClockwise
                : TalonFXInvertType.Clockwise);
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the
                                                                                   // Falcon is the falcon's encoder

        mDriveMotor.configClosedloopRamp(0.5);
        mDriveMotor.configOpenloopRamp(0.5);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Encoder position", this::getEncoderPosition, this::setEncoderPosition);
        builder.addDoubleProperty("Encoder absolute position", this::getEncoderAbsolutePosition,
                this::setEncoderPosition);
        builder.addDoubleProperty("Drive motor velocity error", () -> mDriveMotor.getClosedLoopError(), null);
        builder.addDoubleProperty("Drive motor velocity setpoint", () -> mDriveMotor.getClosedLoopTarget(), null);
        builder.addDoubleProperty("Drive motor output voltage", () -> mDriveMotor.getMotorOutputVoltage(), null);
        builder.addDoubleProperty("Drive motor output percent", () -> mDriveMotor.getMotorOutputPercent(), null);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                CTREConverter.falconToMeters(
                        mDriveMotor.getSelectedSensorPosition(),
                        DriveMap.kDriveWheelCircumference,
                        DriveMap.kDriveGearRatio),
                getAbsoluteRotation2d());
    }

    public void setDesiredAngle(Rotation2d angle) {
        getController().setSetpoint(MathUtil.angleModulus(angle.getRadians()));
    }

    public void setDesiredSpeed(double speedMetersPerSecond, boolean inHighGear) {

        var desiredVelocity = CTREConverter.MPSToFalcon(speedMetersPerSecond,
                DriveMap.kDriveWheelCircumference, DriveMap.kDriveGearRatio);
        mDriveMotor.set(ControlMode.Velocity,
                CTREConverter.MPSToFalcon(speedMetersPerSecond,
                        DriveMap.kDriveWheelCircumference, DriveMap.kDriveGearRatio));
        // var percentOutput = speedMetersPerSecond /
        // DriveMap.kDriveMaxSpeedMetersPerSecond;

        // if (!inHighGear)
        // percentOutput *= 0.5;

        // mDriveMotor.set(ControlMode.PercentOutput, MathUtil.clamp(
        // percentOutput * DriveMap.kDriveMaxSpeedMetersPerSecond, -1, 1));
    }

    /**
     * Sets the desired state of the module.
     * 
     * @param desiredState The state of the module that we'd like to be at in this
     *                     period
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean inHighGear) {
        // Optimize the state to avoid turning wheels further than 90 degrees
        var encoderRotation = getAbsoluteRotation2d();
        desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

        setDesiredSpeed(desiredState.speedMetersPerSecond, inHighGear);
        setDesiredAngle(desiredState.angle);
    }

    /**
     * Sets the encoder position to a new value
     * 
     * @param newPosition the new position of the encoder
     */
    public void setEncoderPosition(double newPosition) {
        mEncoder.setPosition(newPosition);
    }

    public void stopMotors() {
        mDriveMotor.stopMotor();
        mSteeringMotor.stopMotor();
    }

    public double getEncoderPosition() {
        return mEncoder.getPosition();
    }

    public double getRawSpeed() {
        return mDriveMotor.get();
    }

    public double getVelocityMetersPerSecond() {
        // getSelectedSensorVelocity() returns speed in sensor units per 100ms
        // First, convert to sensor units per 1s. Then, convert to MPS
        return CTREConverter.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), DriveMap.kDriveWheelCircumference,
                DriveMap.kDriveGearRatio);
    }

    public void setEncoderPositionToAbsolute() {
        mEncoder.setPositionToAbsolute();
    }

    public double getEncoderAbsolutePosition() {
        return mEncoder.getAbsolutePosition();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(mEncoder.getPosition());
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        mSteeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(output, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        var currentPositionRadians = getAbsoluteRotation2d().getRadians();
        return currentPositionRadians;
    }

    private Rotation2d getAbsoluteRotation2d() {
        return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition() - mEncoderOffset);
    }
}
