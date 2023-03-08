package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.config.ShoulderMap;
import prime.movers.LazyCANSparkMax;

public class Shoulder extends PIDSubsystem {
    private LazyCANSparkMax shoulder1;
    private LazyCANSparkMax shoulder2;
    private WPI_CANCoder mEncoder;
    private double _lastPIDoutput = 0;

    public Shoulder() {
        super(new PIDController(
            ShoulderMap.AnglePid.kP, 
            ShoulderMap.AnglePid.kI, 
            ShoulderMap.AnglePid.kD));

        shoulder1 = new LazyCANSparkMax(ShoulderMap.kShoulder1Id, MotorType.kBrushless);
        shoulder1.clearFaults();
        shoulder1.restoreFactoryDefaults();
        shoulder1.setIdleMode(IdleMode.kBrake);
        shoulder1.setOpenLoopRampRate(ShoulderMap.kOpenLoopRampRate);

        shoulder2 = new LazyCANSparkMax(ShoulderMap.kShoulder2Id, MotorType.kBrushless);
        shoulder2.restoreFactoryDefaults();
        shoulder2.follow(shoulder1);

        mEncoder = new WPI_CANCoder(ShoulderMap.kEncoderId);

        var pidController = getController();
        pidController.reset();
        pidController.setTolerance(0.1);
        disable();
    }

    public void runShoulder(double speed) {
        speed *= 0.3;
        shoulder1.set(MathUtil.applyDeadband(speed, 0.15));
    }

    public void setShoulderAngle(double angleInDegrees) {
        setSetpoint(angleInDegrees);
    }

    public void zeroShoulderEncoder() {
        mEncoder.setPosition(0);
    }

    public void setPIDEnabled(boolean enabled) {
        if (enabled)
            enable(); 
        else 
            disable();
    }

    public double getLastPIDOutput() {
        return _lastPIDoutput;
    }

    @Override
    public double getMeasurement() {
        return mEncoder.getAbsolutePosition();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        _lastPIDoutput = MathUtil.clamp(output, -0.2, 0.2);
        runShoulder(_lastPIDoutput);
    }

    @Override
    public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Shoulder Position", this::getMeasurement, null);
      builder.addBooleanProperty("PID enabled", super::isEnabled, null);
      builder.addDoubleProperty("Last PID output", this::getLastPIDOutput, null);
    }
}
