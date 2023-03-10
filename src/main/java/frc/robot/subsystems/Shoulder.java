package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.config.ShoulderMap;
import prime.movers.LazyWPITalonSRX;

public class Shoulder extends PIDSubsystem {
    private LazyWPITalonSRX shoulder1;
    private LazyWPITalonSRX shoulder2;
    private WPI_CANCoder mEncoder;
    private double _lastPIDoutput = 0;

    public Shoulder() {
        super(new PIDController(
            ShoulderMap.AnglePid.kP, 
            ShoulderMap.AnglePid.kI, 
            ShoulderMap.AnglePid.kD));

        shoulder1 = new LazyWPITalonSRX(ShoulderMap.kShoulder1Id);
        shoulder1.clearStickyFaults();
        shoulder1.configFactoryDefault();
        shoulder1.setNeutralMode(NeutralMode.Brake);
        shoulder1.configOpenloopRamp(0.5);

        shoulder2 = new LazyWPITalonSRX(ShoulderMap.kShoulder2Id);
        shoulder2.clearStickyFaults();
        shoulder2.configFactoryDefault();
        shoulder2.setNeutralMode(NeutralMode.Brake);
        shoulder2.follow(shoulder1);

        mEncoder = new WPI_CANCoder(ShoulderMap.kEncoderId);

        var pidController = getController();
        pidController.reset();
        pidController.setTolerance(0.1);
        disable();
    }

    /**
     * Run shoulder at low speed with a deadband
     * @param speed
     */
    public void runShoulder(double speed) {
        shoulder1.set(MathUtil.applyDeadband(speed, 0.15));
    }

    public void setShoulderAngle(double angleInDegrees) {
        setSetpoint(angleInDegrees);
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
        _lastPIDoutput = MathUtil.clamp(output, -0.4, 0.4);
        runShoulderNoFilter(_lastPIDoutput);
    }

    @Override
    public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Shoulder Position", this::getMeasurement, null);
      builder.addBooleanProperty("PID enabled", super::isEnabled, null);
      builder.addDoubleProperty("Last PID output", this::getLastPIDOutput, null);
    }
}
