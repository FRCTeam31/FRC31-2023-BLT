package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;
import prime.movers.LazySolenoid;
import prime.movers.LazyWPITalonSRX;
import frc.robot.config.WristMap;

public class Wrist extends SubsystemBase {
    private LazyCANSparkMax wrist1;
    private LazyCANSparkMax wrist2;
    private LazySolenoid wristActuator; 
    private LazyWPITalonSRX forearm;
    


    public Wrist() {
        wrist1 = new LazyCANSparkMax(WristMap.kWrist1Id, MotorType.kBrushless);
        wrist1.restoreFactoryDefaults();
        
        wrist2 = new LazyCANSparkMax(WristMap.kWrist2Id, MotorType.kBrushless);
        wrist2.restoreFactoryDefaults();

        wristActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId);
        forearm = new LazyWPITalonSRX(WristMap.kForearmId);
    }

    public void intakeCone() {
        wrist1.set(WristMap.kIntakeConeSpeed);
        wrist2.set(WristMap.kIntakeConeSpeed);
    }

    public void intakeCube() {
        wrist1.set(WristMap.kIntakeCubeSpeed);
        wrist2.set(WristMap.kIntakeCubeSpeed);
    }

    public void ejectCone() {
        wrist1.set(WristMap.kEjectConeSpeed);
        wrist2.set(WristMap.kEjectConeSpeed);
    }

    public void ejectCube() {
        wrist1.set(WristMap.kEjectCubeSpeed);
        wrist2.set(WristMap.kEjectCubeSpeed);
    }

    public void stopIntake() {
        wrist1.stopMotor();
        wrist2.stopMotor();
    }

    public void toggleWrist() {
        wristActuator.toggle();
    }

    public void setWrist(boolean up) {
        wristActuator.set(up);
    }

    public boolean getWrist() {
        return wristActuator.get();
    }

    public void pushAndPull(int direction){
        forearm.set(WristMap.kPushAndPullSpeed * direction);

    }

    public void stopForearm(){
        forearm.stopMotor();
    }


}
