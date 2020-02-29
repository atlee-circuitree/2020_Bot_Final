/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.ADIS16448_IMU;
import frc.robot.drivers.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;

public class IMUSubsystem extends SubsystemBase {
	private final ADIS16448_IMU m_imu = new ADIS16448_IMU(IMUAxis.kZ, SPI.Port.kOnboardCS0, 4, true, 2, 1);

  /**
   * Creates a new Lidar.
   */
  public IMUSubsystem() {

  }

  

  @Override
  public void periodic() {
	SmartDashboard.putNumber("XCompAngle", m_imu.getXComplementaryAngle());
  }

  /**
	 * 
	 * @return Tilt Angle in degrees
	 */
	public double getTilt() {
		return m_imu.getXComplementaryAngle();
	}


}
