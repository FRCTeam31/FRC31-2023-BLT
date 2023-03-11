package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.config.foram_map;

public class forArme {
private  LazyTalonSRX foeram;

public void Arme(command runOnce) {

foeram = new LazyTalonSRX(foram_map.kfoeramId, MotorType.kBrushless);
foeram.clearFaults();
foeram.restoreFactoryDefaults();
foeram.setIdleMode(IdleMode.kBrake);
foeram.setOpenLoopRampRate(foram_map.kOpenLoopRampRate);
}

public Object setSetpoint(double angle) {
    return null;
}

public Object runforArme(double rawAxis) {
    return null;
}

public Object setPIDEnabled(boolean enabled) {
    return null;
}

}
