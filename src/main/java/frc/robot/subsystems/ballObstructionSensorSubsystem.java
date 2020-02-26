package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 
 
public class ballObstructionSensorSubsystem extends SubsystemBase {

DigitalInput input;

 
public ballObstructionSensorSubsystem() {

  input = new DigitalInput(Constants.PORT_OBSTRUCTION_SENSOR);

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
