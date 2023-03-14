package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;
import prime.movers.LazySolenoid;
import frc.robot.config.WristMap;

public class Wrist extends SubsystemBase {
    private LazyCANSparkMax wrist1;
    private LazySolenoid wristActuator; 
    
    


    public Wrist() {
        wrist1 = new LazyCANSparkMax(WristMap.kWrist1Id, MotorType.kBrushless);
        wrist1.restoreFactoryDefaults();
        
        
     

        wristActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId);
        
    }

    @Deprecated
    public void intakeCone() {
       runMotors(WristMap.kIntakeConeSpeed);  
     }

     @Deprecated
    public void intakeCube() {
        runMotors(WristMap.kIntakeCubeSpeed);
    }

    @Deprecated
    public void ejectCone() {
        runMotors(WristMap.kEjectConeSpeed);
    
    }

    @Deprecated
    public void ejectCube() {
        runMotors(WristMap.kEjectCubeSpeed);
    }

    public void stopIntake() {
        wrist1.stopMotor();
        
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

    

    public void runMotors(double speed){
        wrist1.set(speed * 0.60);
        
    }
    
    

}
