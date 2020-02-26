/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lidar extends SubsystemBase {
  public final static int BAUD = 115200;
  private SerialPort serial;
  volatile private int distance;

  /**
   * Creates a new Lidar.
   */
  public Lidar() {
    connect();
  }

  public void connect() {
    if (serial == null) {
      try {
        serial = new SerialPort(BAUD, SerialPort.Port.kUSB);
        System.out.println("Lidar connected on port 0");
      } catch (Exception e) {
        System.err.println("LIDAR Cannot Connect on Port 0"); // serial will be null here
        try {
          serial = new SerialPort(BAUD, SerialPort.Port.kUSB1);
          System.out.println("Lidar connected on port 1");
        } catch (Exception e2) {
          System.err.println("LIDAR Cannot Connect on Port 1"); // serial will be null here
          try {
            serial = new SerialPort(BAUD, SerialPort.Port.kUSB2);
            System.out.println("Lidar connected on port 2");
          } catch (Exception e3) {
            System.err.println("LIDAR Cannot Connect on Port 2"); // serial will be null here
          }
        }
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(serial != null) {
      updateDistance();
      //updateSmartDashboard();
    }
  }

  /**
	 * Reads current saved distance
	 * 
	 * @return distance in CM
	 */
	public int getDistance() {
		return distance;
	}

	/**
	 * Whether more than two 
	 * bytes are available to read
	 * 
	 * @return
	 */
	private boolean bytesAvailable() {
		return isConnected() && serial.getBytesReceived() > 0;
  }
  
  /**
	 * Reads from Serial data buffer until no data
	 * exists, and sets the value of distance
	 */
	private void updateDistance() {
		byte lastByte = 0;
		int frameIndex = -100;
		
		while(bytesAvailable()) {
			
			byte read = readByte();
			//System.out.write(read);
			if(read == 0x59 && lastByte == 0x59)
			{
				//System.out.println("Lidar begin frame");
				frameIndex = 1;
				distance = 0;
			}
			else{
				frameIndex++;
				if(frameIndex == 3)
				{
					distance = (((int)read & 0xFF) << 8) + (int)(lastByte & 0xFF);
					//System.out.print(String.format("0x%08X",read));
					//System.out.print(" ");
					System.out.println(String.format("0x%08X",lastByte));
					System.out.println((int)distance);
					
				}
			}
			
			// if (read == 13) {
			// 	//CR
			// } else if (read == 10) {
			// 	//LF, last
			// 	distance = distanceBuffer;
			// 	distanceBuffer = 0;
			// } else if (read >= 48 && read <= 57) {
			// 	//NUM
			// 	distanceBuffer *= 10;
			// 	distanceBuffer += (int)read - 48;
			// } else {
			// 	//System.err.println("xxLIDAR Read Invalid Character");
			// }
			lastByte = read;
		} //end while
  } //end method
  
  public boolean isConnected() {
		return serial != null;
	}

	private byte readByte() {
		byte b = 0;
		if(bytesAvailable()) {
			return serial.read(1)[0];
		} 
		return b;
  }
  
  /**
	 * Updates the NetworkTable with the 
	 * most recent distance information
	 */
	public void updateSmartDashboard(	) {
		SmartDashboard.putNumber("LIDAR Distance", distance);
	}
}
