package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
 
public class ballObstructionSensorSubsystem extends SubsystemBase {
 
DigitalInput input;
 
  
public ballObstructionSensorSubsystem() {
 
input = new DigitalInput(0);
 
}
 
public boolean isObstructed() {
 
    return !input.get();
}
 
public boolean isNotObstructed() {
 
    return input.get();
 
}
 
  @Override
  public void periodic() {
    
   // System.out.println(input.get());
 
  }
}
